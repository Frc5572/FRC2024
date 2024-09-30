use class::JavaClass;
use proc_macro::TokenStream as StdTokenStream;
use syn::{braced, parenthesized, parse::{Parse, ParseStream}, parse_macro_input, token, Ident, LitStr, Token};

pub(crate) mod class;

#[proc_macro]
pub fn java_util(input: StdTokenStream) -> StdTokenStream {
    let x = parse_macro_input!(input as Parser);
    
    let x = format!("{:#?}", x);

    quote::quote! {
        fn test() {
            println!("{}", #x);
        }
    }.into()
}

#[derive(Debug)]
struct Parser {
    classes: Vec<JavaClass>,
}

impl Parse for Parser {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let mut classes = Vec::new();
        while !input.is_empty() {
            if input.peek(Token![extern]) {
                let extern_block;
                let _ = braced!(extern_block in input);
                // TODO native block
            } else {
                let class: JavaClass = input.parse()?;
                classes.push(class);
            }
        }
        return Ok(Self {
            classes,
        })
    }
}
