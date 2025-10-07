use proc_macro::TokenStream;
use proc_macro2::{Span, TokenStream as TokenStream2};
use quote::{ToTokens, format_ident, quote};
use syn::{
    Error, Field, Fields, FieldsNamed, FieldsUnnamed, Ident, ItemStruct, Token, Type, Visibility,
    parse::Parse, parse_quote, punctuated::Punctuated,
};

/// # Example
/// ```rust
/// use rust_livo2_macros::uncertainties;
/// #[uncertainties]
/// struct PointUncertainty {
///     pub distance: DistanceUncertainty, // publish this field will generate a view method
///     direction: DirectionUncertainty,
/// }
/// ```
#[proc_macro_attribute]
pub fn uncertainties(_attr: TokenStream, input: TokenStream) -> TokenStream {
    syn::parse_macro_input!(input as ItemUncertainties)
        .to_token_stream()
        .into()
}

struct ItemUncertainties {
    item: ItemStruct,
    field_sum_ty_alias: Ident,
    field_sum_ty: Type,
    folded_field_tys: Vec<Type>,
    origin_fields: Punctuated<Field, Token![,]>,
}

impl Parse for ItemUncertainties {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let mut item: ItemStruct = input.parse()?;

        let item_ident = &item.ident;

        let Fields::Named(FieldsNamed { named: fields, .. }) = item.fields else {
            return Err(Error::new(input.span(), "Fields must be named"));
        };
        let first_field = fields
            .first()
            .ok_or(Error::new(input.span(), "No fields found"))?;
        let first_ty = &first_field.ty;

        let uncertainty_element_ty = quote!(
            <#first_ty as crate::uncertain::Uncertainty>::Element
        );

        let uncertain_zero_ty: Type =
            parse_quote!(crate::uncertain::UncertaintyZero<#uncertainty_element_ty>);

        let mut folded_field_tys = fields
            .iter()
            .map(|field| field.ty.clone())
            .scan(uncertain_zero_ty.clone(), |left, right| {
                let output: Type = parse_quote!(
                    <#left as crate::uncertain::UncertaintyAdd<#right>>::Output
                );
                *left = output.clone();
                Some(output)
            })
            .collect::<Vec<_>>();

        let field_sum_ty = folded_field_tys
            .pop()
            .ok_or(Error::new(input.span(), "No fields found"))?;

        folded_field_tys.insert(0, uncertain_zero_ty);

        let field_sum_ty_alias = format_ident!("{}Matrix", item_ident);

        let item_fields: FieldsUnnamed = parse_quote! {
            (#field_sum_ty_alias)
        };
        item.fields = Fields::Unnamed(item_fields);
        item.semi_token = Some(Token![;](Span::call_site()));

        Ok(Self {
            item,
            field_sum_ty_alias,
            field_sum_ty,
            folded_field_tys,
            origin_fields: fields,
        })
    }
}

impl ToTokens for ItemUncertainties {
    fn to_tokens(&self, tokens: &mut TokenStream2) {
        let Self {
            item,
            field_sum_ty_alias: sum_field_ty_alias,
            field_sum_ty: sum_field_ty,
            folded_field_tys,
            origin_fields,
        } = self;
        let item_ident = &item.ident;

        item.to_tokens(tokens);

        quote! {
            type #sum_field_ty_alias = #sum_field_ty;
        }
        .to_tokens(tokens);

        quote! {
            impl crate::uncertain::Uncertainty for #item_ident {
                type Element = <#sum_field_ty_alias as crate::uncertain::Uncertainty>::Element;
                type Dim = <#sum_field_ty_alias as crate::uncertain::Uncertainty>::Dim;
            }
        }
        .to_tokens(tokens);

        let pub_field_view_fns = origin_fields
            .iter()
            .zip(folded_field_tys)
            .filter(|(field,_)| matches!(field.vis, Visibility::Public(_)))
            .filter_map(|(field, folded_field_ty)| {
                let ident = field.ident.as_ref()?;
                let ty = &field.ty;

                let fn_view_ident = format_ident!("view_{}", ident);
                Some(quote! {
                    pub fn #fn_view_ident(&self) -> nalgebra::MatrixView<
                        <Self as crate::uncertain::Uncertainty>::Element,
                        <#ty as crate::uncertain::Uncertainty>::Dim,
                        <#ty as crate::uncertain::Uncertainty>::Dim,
                        nalgebra::Const<1>,
                        <Self as crate::uncertain::Uncertainty>::Dim,
                    > {
                        let start = <<#folded_field_ty as crate::uncertain::Uncertainty>::Dim as nalgebra::dimension::DimName>::dim();
                        self.0.fixed_view(start, start)
                    }
                })
            });

        quote! {
            impl #item_ident {
                #(#pub_field_view_fns)*
            }
        }
        .to_tokens(tokens);

        quote! {
            impl std::ops::Deref for #item_ident {
                type Target = #sum_field_ty_alias;
                fn deref(&self) -> &Self::Target {
                    &self.0
                }
            }

            impl std::ops::DerefMut for #item_ident {
                fn deref_mut(&mut self) -> &mut Self::Target {
                    &mut self.0
                }
            }
        }
        .to_tokens(tokens);

        quote! {
            impl From<#sum_field_ty_alias> for #item_ident {
                fn from(matrix: #sum_field_ty_alias) -> Self {
                    Self(matrix)
                }
            }
        }
        .to_tokens(tokens);
    }
}

// fn uncertainty_add_output(left: TokenStream2, right: TokenStream2) -> TokenStream2 {
//     quote! {
//         <#left as crate::uncertain::UncertaintyAdd<#right>>::Output
//     }
// }
