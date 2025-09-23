use proc_macro::TokenStream;
use proc_macro2::{Span, TokenStream as TokenStream2};
use quote::{ToTokens, format_ident, quote};
use syn::{
    Error, Fields, FieldsNamed, FieldsUnnamed, ItemStruct, Token, Visibility, parse::Parse,
    parse_quote,
};

/// # Example
/// ```rust
/// use rust_livo2_macros::uncertainties;
/// #[uncertainties]
/// struct PointUncertainty {
///     pub distance: DistanceUncertainty,
///     direction: DirectionUncertainty,
/// }
/// ```
#[proc_macro_attribute]
pub fn uncertainties(_attr: TokenStream, input: TokenStream) -> TokenStream {
    let ItemUncertainties {
        item,
        type_alias_item: type_alias,
        uncertainty_impl,
        view_impl,
        deref_impl,
        convert_impl,
    } = syn::parse_macro_input!(input as ItemUncertainties);

    quote! {
        #item
        #type_alias
        #uncertainty_impl
        #view_impl
        #deref_impl
        #convert_impl
    }
    .into()
}

struct ItemUncertainties {
    item: ItemStruct,
    type_alias_item: TokenStream2,
    uncertainty_impl: TokenStream2,
    view_impl: TokenStream2,
    deref_impl: TokenStream2,
    convert_impl: TokenStream2,
}

impl Parse for ItemUncertainties {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let mut item: ItemStruct = input.parse()?;
        let item_ident = &item.ident;
        let Fields::Named(FieldsNamed { named: fields, .. }) = item.fields else {
            return Err(Error::new(input.span(), "Fields must be named"));
        };

        let sum_field_ty = fields
            .iter()
            .map(|field| field.ty.clone().to_token_stream())
            .reduce(uncertainty_add_output)
            .ok_or(Error::new(input.span(), "No fields found"))?;

        let sum_field_ty_alias = format_ident!("{}Matrix", item_ident);
        let type_alias_item = quote! {
            type #sum_field_ty_alias = #sum_field_ty;
        };

        let item_fields: FieldsUnnamed = parse_quote! {
            (#sum_field_ty_alias)
        };
        item.fields = Fields::Unnamed(item_fields);
        item.semi_token = Some(Token![;](Span::call_site()));

        let pub_field_view_fns = fields
            .into_iter()
            .filter(|field| matches!(field.vis, Visibility::Public(_)))
            .filter_map(|field| {
                let ident = field.ident?;
                let ty = field.ty;
                Some((ident, ty))
            })
            .enumerate()
            .map(|(i, (ident, ty))| {
                let fn_view_ident = format_ident!("view_{}", ident);
                let view_start_ident_last = format_ident!("view_start_{}", i);
                let view_start_ident = format_ident!("view_start_{}", i + 1);
                quote! {
                    #[allow(unused)]
                    #[inline]
                    fn #view_start_ident() -> usize {
                        <<#ty as crate::uncertain::Uncertainty>::Dim as nalgebra::dimension::DimName>::dim()
                        + Self::#view_start_ident_last()
                    }

                    pub fn #fn_view_ident(&self) -> nalgebra::MatrixView<
                        <Self as crate::uncertain::Uncertainty>::Element,
                        <#ty as crate::uncertain::Uncertainty>::Dim,
                        <#ty as crate::uncertain::Uncertainty>::Dim,
                        nalgebra::Const<1>,
                        <Self as crate::uncertain::Uncertainty>::Dim,
                    > {
                        let start = Self::#view_start_ident_last();
                        self.0.fixed_view(start, start)
                    }
                }
            });

        let uncertainty_impl = quote! {
            impl crate::uncertain::Uncertainty for #item_ident {
                type Element = <#sum_field_ty_alias as crate::uncertain::Uncertainty>::Element;
                type Dim = <#sum_field_ty_alias as crate::uncertain::Uncertainty>::Dim;
            }
        };

        let view_impl = quote! {
            impl #item_ident {
                #[inline]
                fn view_start_0() -> usize {
                    0
                }
                #(#pub_field_view_fns)*
            }
        };

        let deref_impl = quote! {
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
        };

        let convert_impl = quote! {
            impl From<#sum_field_ty_alias> for #item_ident {
                fn from(matrix: #sum_field_ty_alias) -> Self {
                    Self(matrix)
                }
            }
        };

        Ok(Self {
            item,
            type_alias_item,
            uncertainty_impl,
            view_impl,
            deref_impl,
            convert_impl,
        })
    }
}

fn uncertainty_add_output(left: TokenStream2, right: TokenStream2) -> TokenStream2 {
    quote! {
        <#left as crate::uncertain::UncertaintyAdd<#right>>::Output
    }
}
