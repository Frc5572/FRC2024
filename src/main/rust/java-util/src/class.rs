use syn::{braced, parenthesized, parse::Parse, Ident, LitStr, Token};

#[derive(Debug)]
pub struct JavaClass {
    pub name: Ident,
    pub internal_name: String,
    pub fields: Vec<JavaField>,
    pub methods: Vec<JavaMethod>,
}

impl Parse for JavaClass {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let name: Ident = input.parse()?;
        let internal_name;
        let _paren = parenthesized!(internal_name in input);
        let internal_name: LitStr = internal_name.parse()?;
        let internal_name = internal_name.value();
        let body;
        let _brace = braced!(body in input);
        let body = body.parse_terminated(parse_member, Token![,])?;
        let mut fields = Vec::new();
        let mut methods = Vec::new();
        for item in body {
            match item {
                Member::Field(java_field) => fields.push(java_field),
                Member::Method(java_method) => methods.push(java_method),
            }
        }
        Ok(Self {
            name,
            internal_name,
            fields,
            methods,
        })
    }
}

#[derive(Debug)]
pub struct JavaField {
    pub is_static: bool,
    pub get_name: Ident,
    pub set_name: Option<Ident>,
    pub java_name: String,
    pub ty: TypeDescriptor,
}

impl Parse for JavaField {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let is_static: Option<Token![static]> = input.parse()?;
        let is_static = is_static.is_some();
        let content;
        let _paren = parenthesized!(content in input);
        let get_name: Ident = content.parse()?;
        let _comma: Token![,] = content.parse()?;
        let set_name = if content.peek(Token![_]) {
            let _set_name: Token![_] = content.parse()?;
            None
        } else {
            let set_name: Ident = content.parse()?;
            Some(set_name)
        };
        let _comma: Token![,] = content.parse()?;
        let java_name_raw: LitStr = content.parse()?;
        let java_name = java_name_raw.value();
        let _comma: Token![,] = content.parse()?;
        let descriptor_raw: LitStr = content.parse()?;
        let descriptor = descriptor_raw.value();
        let ty = TypeDescriptor::parse(&descriptor).map_err(|x| syn::Error::new(descriptor_raw.span(), x))?.1;
        let _comma: Option<Token![,]> = content.parse()?;
        Ok(JavaField { is_static, get_name, set_name, java_name, ty })
    }
}

#[derive(Debug)]
pub struct JavaMethod {
    pub is_static: bool,
    pub name: Ident,
    pub java_name: String,
    pub args: Vec<TypeDescriptor>,
    pub ret: Option<TypeDescriptor>,
}

impl Parse for JavaMethod {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let is_static: Option<Token![static]> = input.parse()?;
        let is_static = is_static.is_some();
        let _fn: Token![fn] = input.parse()?;
        let content;
        let _paren = parenthesized!(content in input);
        let name: Ident = content.parse()?;
        let _comma: Token![,] = content.parse()?;
        let java_name_raw: LitStr = content.parse()?;
        let java_name = java_name_raw.value();
        let _comma: Token![,] = content.parse()?;
        let descriptor_raw: LitStr = content.parse()?;
        let descriptor = descriptor_raw.value();
        let (args, ret) = match TypeDescriptor::parse_method(&descriptor) {
            Err(x) => return Err(syn::Error::new(descriptor_raw.span(), x)),
            Ok((_, args, ret)) => {
                (args, ret)
            }
        };
        let _comma: Option<Token![,]> = content.parse()?;
        Ok(JavaMethod { is_static, name, java_name, args, ret })
    }
}

enum Member {
    Field(JavaField),
    Method(JavaMethod),
}

fn parse_member(input: syn::parse::ParseStream) -> syn::Result<Member> {
    if input.peek(Token![fn]) || input.peek2(Token![fn]) {
        return Ok(Member::Method(JavaMethod::parse(input)?))
    } else {
        return Ok(Member::Field(JavaField::parse(input)?))
    }
}

#[derive(Debug)]
pub enum TypeDescriptor {
    Byte,
    Char,
    Double,
    Float,
    Int,
    Long,
    Short,
    Boolean,
    String,
    Class,
    Object(String),
    Array(Box<TypeDescriptor>),
}

impl TypeDescriptor {
    fn parse<'a>(input: &'a str) -> Result<(&'a str, Self), String> {
        match input.chars().next() {
            Some('B') => Ok((&input[1..], Self::Byte)),
            Some('C') => Ok((&input[1..], Self::Char)),
            Some('D') => Ok((&input[1..], Self::Double)),
            Some('F') => Ok((&input[1..], Self::Float)),
            Some('I') => Ok((&input[1..], Self::Int)),
            Some('J') => Ok((&input[1..], Self::Long)),
            Some('S') => Ok((&input[1..], Self::Short)),
            Some('Z') => Ok((&input[1..], Self::Boolean)),
            Some('L') => {
                if let Some(x) = input.char_indices().find(|x| x.1 == ';') {
                    let res = input[1 .. x.0].to_string();
                    let leftover = &input[x.0 + 1 ..];
                    match res.as_str() {
                        "java/lang/String" => Ok((leftover, Self::String)),
                        "java/lang/Class" => Ok((leftover, Self::Class)),
                        _ => Ok((leftover, Self::Object(res))),
                    }
                } else {
                    return Err(format!("Expected `;`."))
                }
            },
            Some('[') => {
                let (leftover, next) = Self::parse(&input[1..])?;
                Ok((leftover, Self::Array(Box::new(next))))
            },
            None => Err("Expected more contents!".to_string()),
            Some(x) => Err(format!("Expected one of `BCDFIJSZL[`, instead got '{}'.", x))
        }
    }

    fn parse_method<'a>(input: &'a str) -> Result<(&'a str, Vec<Self>, Option<Self>), String> {
        match input.chars().next() {
            None => return Err("Expected more contents!".to_string()),
            Some('(') => {},
            Some(x) => return Err(format!("Expected `(`, instead got '{}'.", x)),
        }
        let mut input = &input[1..];
        let mut args = Vec::new();
        loop {
            match input.chars().next() {
                Some(')') => break,
                None => return Err(format!("Expected `)`.")),
                _ => {
                    let (i2, arg) = Self::parse(input)?;
                    input = i2;
                    args.push(arg);
                }
            }
        }
        let input = &input[1..];
        let (input, ret) = match input.chars().next() {
            Some('V') => (input, None),
            None => return Err(format!("Expected return type or void!")),
            _ => {
                let (a, b) = Self::parse(input)?;
                (a, Some(b))
            }
        };
        Ok((input, args, ret))
    }
}
