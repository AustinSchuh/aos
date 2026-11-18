//! Conditions on a channel's messages: what MCP's `wait_for` waits for.
//!
//! A condition is an expression in a subset of CEL's syntax, which models already write, over
//! the fields of the channel's message:
//!
//! ```text
//! abs(wrap(heading - goal)) > radians(5) && state == 'INTAKING'
//! has(left) ? left.velocity > 1 : stale(500ms)
//! ```
//!
//! [`Condition::compile`] resolves every field path against the channel's reflection schema,
//! and checks types, so a mistake is found when the condition is made, not on the first
//! message.  Evaluating one reads the message's bytes directly, bounds checked, without
//! converting it or allocating.
//!
//! Operators, lowest precedence first: `?:` (right associative); `||`; `&&`; `==` `!=`;
//! `<` `<=` `>` `>=` (comparisons don't chain); `+` `-`; `*` `/` `%`; unary `!` `-`; and calls,
//! `.field` and `[index]`.  Integers are exact (`int64` and `uint64` alike), `/` always gives a
//! float, and anything mixed with a float is a float.  Comparisons follow IEEE: anything but
//! `!=` involving `nan` is false.  Enums compare with their values' names, as strings.
//!
//! A field that isn't set reads as its default, as in the generated code; `has()` tells the
//! two apart.  A table, string or vector that isn't there, or an index past the end, is an
//! error, which makes the condition false for that message.

use std::fmt;

use flatbuffers_reflection::reflection::{BaseType, Object, Schema};

/// Why a condition didn't compile, and where in it.
#[derive(Clone, Debug, PartialEq)]
pub struct CompileError {
    /// Byte offsets into the condition of what's wrong.
    pub start: usize,
    pub end: usize,
    pub message: String,
}

impl CompileError {
    fn new(span: Span, message: impl Into<String>) -> Self {
        Self {
            start: span.0,
            end: span.1,
            message: message.into(),
        }
    }

    /// The condition with what's wrong underlined, and why.
    pub fn render(&self, source: &str) -> String {
        let before = source.get(..self.start).unwrap_or(source).chars().count();
        let width = source
            .get(self.start..self.end.max(self.start))
            .map_or(1, |s| s.chars().count().max(1));
        format!(
            "{source}\n{}{} {}",
            " ".repeat(before),
            "^".repeat(width),
            self.message
        )
    }
}

impl fmt::Display for CompileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "at {}: {}", self.start, self.message)
    }
}

/// Why a condition couldn't be evaluated on a message: it counts as false.
#[derive(Clone, Debug, PartialEq)]
pub struct EvalError(pub String);

impl fmt::Display for EvalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

type Span = (usize, usize);

// ---------------------------------------------------------------------------------------------
// Tokens.

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Int(i128),
    Float(f64),
    Str(String),
    Ident(String),
    /// Nanoseconds.
    Duration(i64),
    Punct(&'static str),
    End,
}

#[derive(Clone, Debug)]
struct Token {
    tok: Tok,
    span: Span,
}

const PUNCTUATION: [&str; 22] = [
    "&&", "||", "==", "!=", "<=", ">=", "(", ")", "[", "]", ".", ",", "?", ":", "!", "<", ">", "+",
    "-", "*", "/", "%",
];

fn lex(source: &str) -> Result<Vec<Token>, CompileError> {
    let bytes = source.as_bytes();
    let mut tokens = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i];
        if c.is_ascii_whitespace() {
            i += 1;
            continue;
        }
        let start = i;
        if c.is_ascii_digit() || (c == b'.' && bytes.get(i + 1).is_some_and(u8::is_ascii_digit)) {
            let mut float = false;
            while i < bytes.len() && bytes[i].is_ascii_digit() {
                i += 1;
            }
            if i < bytes.len() && bytes[i] == b'.' {
                float = true;
                i += 1;
                while i < bytes.len() && bytes[i].is_ascii_digit() {
                    i += 1;
                }
            }
            if i < bytes.len() && (bytes[i] == b'e' || bytes[i] == b'E') {
                let mut j = i + 1;
                if j < bytes.len() && (bytes[j] == b'+' || bytes[j] == b'-') {
                    j += 1;
                }
                if j < bytes.len() && bytes[j].is_ascii_digit() {
                    float = true;
                    i = j;
                    while i < bytes.len() && bytes[i].is_ascii_digit() {
                        i += 1;
                    }
                }
            }
            let number = &source[start..i];
            let unit_start = i;
            while i < bytes.len() && bytes[i].is_ascii_alphabetic() {
                i += 1;
            }
            let unit = &source[unit_start..i];
            let span = (start, i);
            let tok =
                if unit.is_empty() {
                    if float {
                        Tok::Float(number.parse().map_err(|_| {
                            CompileError::new(span, format!("{number} isn't a number"))
                        })?)
                    } else {
                        Tok::Int(
                            number.parse().map_err(|_| {
                                CompileError::new(span, format!("{number} is too big"))
                            })?,
                        )
                    }
                } else {
                    let scale = match unit {
                        "ns" => 1.0,
                        "us" => 1e3,
                        "ms" => 1e6,
                        "s" => 1e9,
                        _ => {
                            return Err(CompileError::new(
                                (unit_start, i),
                                format!("{unit} isn't a unit: durations are in ns, us, ms or s"),
                            ))
                        }
                    };
                    let value: f64 = number
                        .parse()
                        .map_err(|_| CompileError::new(span, format!("{number} isn't a number")))?;
                    Tok::Duration((value * scale).round() as i64)
                };
            tokens.push(Token { tok, span });
            continue;
        }
        if c.is_ascii_alphabetic() || c == b'_' {
            while i < bytes.len() && (bytes[i].is_ascii_alphanumeric() || bytes[i] == b'_') {
                i += 1;
            }
            tokens.push(Token {
                tok: Tok::Ident(source[start..i].to_string()),
                span: (start, i),
            });
            continue;
        }
        if c == b'\'' || c == b'"' {
            let mut text = String::new();
            i += 1;
            loop {
                let Some(&b) = bytes.get(i) else {
                    return Err(CompileError::new((start, i), "This string isn't closed"));
                };
                if b == c {
                    i += 1;
                    break;
                }
                if b == b'\\' {
                    let escaped = match bytes.get(i + 1) {
                        Some(b'n') => '\n',
                        Some(b't') => '\t',
                        Some(b'\\') => '\\',
                        Some(b'\'') => '\'',
                        Some(b'"') => '"',
                        _ => {
                            return Err(CompileError::new(
                                (i, (i + 2).min(bytes.len())),
                                "Unknown escape",
                            ))
                        }
                    };
                    text.push(escaped);
                    i += 2;
                    continue;
                }
                // A whole character, which may be several bytes.
                let ch = source[i..].chars().next().expect("i is on a boundary");
                text.push(ch);
                i += ch.len_utf8();
            }
            tokens.push(Token {
                tok: Tok::Str(text),
                span: (start, i),
            });
            continue;
        }
        if let Some(punct) = PUNCTUATION.iter().find(|p| source[i..].starts_with(*p)) {
            i += punct.len();
            tokens.push(Token {
                tok: Tok::Punct(punct),
                span: (start, i),
            });
            continue;
        }
        let hint = match c {
            b'=' => "Compare with ==",
            b'&' => "Use && for and",
            b'|' => "Use || for or",
            _ => "Unexpected character",
        };
        let width = source[i..].chars().next().map_or(1, char::len_utf8);
        return Err(CompileError::new((i, i + width), hint));
    }
    tokens.push(Token {
        tok: Tok::End,
        span: (source.len(), source.len()),
    });
    Ok(tokens)
}

// ---------------------------------------------------------------------------------------------
// Syntax.

#[derive(Clone, Debug)]
enum Seg {
    Field(String, Span),
    Index(usize, Span),
}

#[derive(Clone, Debug)]
enum Ast {
    Int(i128),
    Float(f64),
    Str(String),
    Bool(bool),
    Duration(i64),
    Path(Vec<Seg>),
    Call(String, Span, Vec<Node>),
    Unary(&'static str, Box<Node>),
    Binary(&'static str, Box<Node>, Box<Node>),
    Ternary(Box<Node>, Box<Node>, Box<Node>),
}

#[derive(Clone, Debug)]
struct Node {
    ast: Ast,
    span: Span,
}

struct Parser {
    tokens: Vec<Token>,
    pos: usize,
}

fn precedence(op: &str) -> Option<u8> {
    Some(match op {
        "||" => 1,
        "&&" => 2,
        "==" | "!=" => 3,
        "<" | "<=" | ">" | ">=" => 4,
        "+" | "-" => 5,
        "*" | "/" | "%" => 6,
        _ => return None,
    })
}

impl Parser {
    fn peek(&self) -> &Token {
        &self.tokens[self.pos]
    }

    fn next(&mut self) -> Token {
        let token = self.tokens[self.pos].clone();
        if self.pos + 1 < self.tokens.len() {
            self.pos += 1;
        }
        token
    }

    fn is(&self, punct: &str) -> bool {
        matches!(self.peek().tok, Tok::Punct(p) if p == punct)
    }

    fn expect(&mut self, punct: &'static str, what: &str) -> Result<Span, CompileError> {
        if self.is(punct) {
            Ok(self.next().span)
        } else {
            Err(CompileError::new(
                self.peek().span,
                format!("Expected {punct} {what}"),
            ))
        }
    }

    fn whole(mut self) -> Result<Node, CompileError> {
        if self.peek().tok == Tok::End {
            return Err(CompileError::new(
                self.peek().span,
                "The condition is empty",
            ));
        }
        let node = self.ternary()?;
        if self.peek().tok != Tok::End {
            return Err(CompileError::new(
                self.peek().span,
                "Expected an operator, or the end of the condition",
            ));
        }
        Ok(node)
    }

    fn ternary(&mut self) -> Result<Node, CompileError> {
        let condition = self.binary(1)?;
        if !self.is("?") {
            return Ok(condition);
        }
        self.next();
        let yes = self.ternary()?;
        self.expect(":", "between ?:'s two choices")?;
        let no = self.ternary()?;
        let span = (condition.span.0, no.span.1);
        Ok(Node {
            ast: Ast::Ternary(Box::new(condition), Box::new(yes), Box::new(no)),
            span,
        })
    }

    fn binary_op(&self) -> Option<(&'static str, u8)> {
        match self.peek().tok {
            Tok::Punct(op) => precedence(op).map(|p| (op, p)),
            _ => None,
        }
    }

    fn binary(&mut self, min: u8) -> Result<Node, CompileError> {
        let mut lhs = self.unary()?;
        while let Some((op, prec)) = self.binary_op() {
            if prec < min {
                break;
            }
            self.next();
            let rhs = self.binary(prec + 1)?;
            if prec == 3 || prec == 4 {
                if let Some((_, next)) = self.binary_op() {
                    if next == prec {
                        return Err(CompileError::new(
                            self.peek().span,
                            "Comparisons don't chain: join them with &&",
                        ));
                    }
                }
            }
            let span = (lhs.span.0, rhs.span.1);
            lhs = Node {
                ast: Ast::Binary(op, Box::new(lhs), Box::new(rhs)),
                span,
            };
        }
        Ok(lhs)
    }

    fn unary(&mut self) -> Result<Node, CompileError> {
        for op in ["!", "-"] {
            if self.is(op) {
                let start = self.next().span.0;
                let operand = self.unary()?;
                let span = (start, operand.span.1);
                return Ok(Node {
                    ast: Ast::Unary(op, Box::new(operand)),
                    span,
                });
            }
        }
        self.primary()
    }

    fn primary(&mut self) -> Result<Node, CompileError> {
        let token = self.next();
        let ast = match token.tok {
            Tok::Int(value) => Ast::Int(value),
            Tok::Float(value) => Ast::Float(value),
            Tok::Str(value) => Ast::Str(value),
            Tok::Duration(value) => Ast::Duration(value),
            Tok::Punct("(") => {
                let inner = self.ternary()?;
                let end = self.expect(")", "to close (")?;
                return Ok(Node {
                    ast: inner.ast,
                    span: (token.span.0, end.1),
                });
            }
            Tok::Ident(name) if name == "true" || name == "false" => Ast::Bool(name == "true"),
            Tok::Ident(name) if self.is("(") => {
                self.next();
                let mut args = Vec::new();
                if !self.is(")") {
                    loop {
                        args.push(self.ternary()?);
                        if !self.is(",") {
                            break;
                        }
                        self.next();
                    }
                }
                let end = self.expect(")", &format!("to close {name}("))?;
                return Ok(Node {
                    ast: Ast::Call(name, token.span, args),
                    span: (token.span.0, end.1),
                });
            }
            Tok::Ident(name) => {
                let mut segs = vec![Seg::Field(name, token.span)];
                let mut end = token.span.1;
                loop {
                    if self.is(".") {
                        self.next();
                        let field = self.next();
                        let Tok::Ident(name) = field.tok else {
                            return Err(CompileError::new(field.span, "Expected a field's name"));
                        };
                        end = field.span.1;
                        segs.push(Seg::Field(name, field.span));
                    } else if self.is("[") {
                        let open = self.next().span.0;
                        let index = self.next();
                        let Tok::Int(value) = index.tok else {
                            return Err(CompileError::new(
                                index.span,
                                "Expected an index: a whole number",
                            ));
                        };
                        let close = self.expect("]", "to close [")?;
                        let Ok(value) = usize::try_from(value) else {
                            return Err(CompileError::new(index.span, "Indexes can't be negative"));
                        };
                        end = close.1;
                        segs.push(Seg::Index(value, (open, close.1)));
                    } else {
                        break;
                    }
                }
                return Ok(Node {
                    ast: Ast::Path(segs),
                    span: (token.span.0, end),
                });
            }
            Tok::End => {
                return Err(CompileError::new(
                    token.span,
                    "The condition ends where a value should be",
                ))
            }
            _ => return Err(CompileError::new(token.span, "Expected a value")),
        };
        Ok(Node {
            ast,
            span: token.span,
        })
    }
}

// ---------------------------------------------------------------------------------------------
// Reading messages.

/// A value read from a message, or computed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Value<'a> {
    Bool(bool),
    Int(i128),
    Float(f64),
    Str(&'a str),
}

impl Value<'_> {
    fn as_f64(self) -> f64 {
        match self {
            Value::Int(v) => v as f64,
            Value::Float(v) => v,
            Value::Bool(v) => v as i32 as f64,
            Value::Str(_) => f64::NAN,
        }
    }
}

impl fmt::Display for Value<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Value::Bool(v) => write!(f, "{v}"),
            Value::Int(v) => write!(f, "{v}"),
            Value::Float(v) => write!(f, "{v}"),
            Value::Str(v) => write!(f, "{v:?}"),
        }
    }
}

fn corrupt() -> EvalError {
    EvalError("The message isn't a valid flatbuffer".into())
}

/// A message's bytes, read with bounds checks.
#[derive(Clone, Copy)]
struct Buf<'a>(&'a [u8]);

impl<'a> Buf<'a> {
    fn bytes<const N: usize>(self, at: usize) -> Result<[u8; N], EvalError> {
        self.0
            .get(at..at.checked_add(N).ok_or_else(corrupt)?)
            .map(|b| b.try_into().expect("N bytes"))
            .ok_or_else(corrupt)
    }

    fn u16(self, at: usize) -> Result<u16, EvalError> {
        Ok(u16::from_le_bytes(self.bytes(at)?))
    }

    fn u32(self, at: usize) -> Result<u32, EvalError> {
        Ok(u32::from_le_bytes(self.bytes(at)?))
    }

    fn i32(self, at: usize) -> Result<i32, EvalError> {
        Ok(i32::from_le_bytes(self.bytes(at)?))
    }

    /// Follows the offset at `at`.
    fn deref(self, at: usize) -> Result<usize, EvalError> {
        at.checked_add(self.u32(at)? as usize).ok_or_else(corrupt)
    }

    /// Where `table`'s field in vtable slot `slot` is, if it's set.
    fn field(self, table: usize, slot: u16) -> Result<Option<usize>, EvalError> {
        let vtable = (table as i64) - (self.i32(table)? as i64);
        let vtable = usize::try_from(vtable).map_err(|_| corrupt())?;
        if slot as usize + 2 > self.u16(vtable)? as usize {
            return Ok(None);
        }
        match self.u16(vtable + slot as usize)? {
            0 => Ok(None),
            offset => Ok(Some(table + offset as usize)),
        }
    }

    fn str(self, at: usize) -> Result<&'a str, EvalError> {
        let len = self.u32(at)? as usize;
        let start = at + 4;
        let bytes = self
            .0
            .get(start..start.checked_add(len).ok_or_else(corrupt)?)
            .ok_or_else(corrupt)?;
        std::str::from_utf8(bytes).map_err(|_| EvalError("A string isn't UTF-8".into()))
    }

    fn scalar(self, at: usize, base: BaseType) -> Result<Value<'a>, EvalError> {
        Ok(match base {
            BaseType::Bool => Value::Bool(self.bytes::<1>(at)?[0] != 0),
            BaseType::Byte => Value::Int(i8::from_le_bytes(self.bytes(at)?) as i128),
            BaseType::UByte | BaseType::UType => Value::Int(self.bytes::<1>(at)?[0] as i128),
            BaseType::Short => Value::Int(i16::from_le_bytes(self.bytes(at)?) as i128),
            BaseType::UShort => Value::Int(self.u16(at)? as i128),
            BaseType::Int => Value::Int(self.i32(at)? as i128),
            BaseType::UInt => Value::Int(self.u32(at)? as i128),
            BaseType::Long => Value::Int(i64::from_le_bytes(self.bytes(at)?) as i128),
            BaseType::ULong => Value::Int(u64::from_le_bytes(self.bytes(at)?) as i128),
            BaseType::Float => Value::Float(f32::from_le_bytes(self.bytes(at)?) as f64),
            BaseType::Double => Value::Float(f64::from_le_bytes(self.bytes(at)?)),
            _ => return Err(EvalError("Not a scalar".into())),
        })
    }
}

/// What a step of a path leads to.
#[derive(Clone, Copy, Debug)]
enum Kind {
    /// A scalar; in a table, read as `default` when it isn't set, or absent if it's optional.
    Scalar(BaseType, Option<DefaultValue>),
    String,
    Table,
    Struct,
    Vector,
    /// A struct's fixed-length array.
    Array(usize),
}

#[derive(Clone, Copy, Debug)]
enum DefaultValue {
    Int(i64),
    Float(f64),
    Bool(bool),
}

#[derive(Clone, Copy, Debug)]
enum Step {
    /// A field of a table, in this vtable slot.
    TableField { slot: u16, kind: Kind },
    /// A field of a struct, this many bytes in.
    StructField { offset: u16, kind: Kind },
    /// An element of a vector or array, each `size` bytes.
    Index {
        index: usize,
        size: usize,
        kind: Kind,
    },
}

/// Where a path has got to.
#[derive(Clone, Copy)]
enum Place<'a> {
    Table(usize),
    Struct(usize),
    /// A vector's or array's elements, and how many.
    Elements(usize, usize),
    Value(Value<'a>),
}

/// What a path found.
enum Found<'a> {
    /// Step `.0` found nothing there.
    Absent(usize),
    /// A scalar that isn't set, so reads as its default.
    Default(Value<'a>),
    /// Step `.0` indexed past the end of `.1` elements.
    PastEnd(usize, usize),
    Here(Place<'a>),
}

/// A field path, resolved against the schema.
#[derive(Clone, Debug)]
struct Path {
    text: String,
    /// Where each step's text ends, for saying which part was missing.
    ends: Vec<usize>,
    steps: Vec<Step>,
}

impl Path {
    fn walk<'a>(&self, data: &'a [u8]) -> Result<Found<'a>, EvalError> {
        let buf = Buf(data);
        let mut place = Place::Table(buf.deref(0)?);
        for (i, step) in self.steps.iter().enumerate() {
            let (at, kind) = match (*step, place) {
                (Step::TableField { slot, kind }, Place::Table(table)) => {
                    match buf.field(table, slot)? {
                        Some(at) => (at, kind),
                        None => {
                            return Ok(match kind {
                                Kind::Scalar(_, Some(default)) => Found::Default(match default {
                                    DefaultValue::Int(v) => Value::Int(v as i128),
                                    DefaultValue::Float(v) => Value::Float(v),
                                    DefaultValue::Bool(v) => Value::Bool(v),
                                }),
                                _ => Found::Absent(i),
                            })
                        }
                    }
                }
                (Step::StructField { offset, kind }, Place::Struct(start)) => {
                    (start + offset as usize, kind)
                }
                (Step::Index { index, size, kind }, Place::Elements(start, len)) => {
                    if index >= len {
                        return Ok(Found::PastEnd(i, len));
                    }
                    (start + index * size, kind)
                }
                _ => unreachable!("Paths are checked against the schema"),
            };
            place = match kind {
                Kind::Scalar(base, _) => Place::Value(buf.scalar(at, base)?),
                Kind::String => Place::Value(Value::Str(buf.str(buf.deref(at)?)?)),
                Kind::Table => Place::Table(buf.deref(at)?),
                Kind::Struct => Place::Struct(at),
                Kind::Vector => {
                    let start = buf.deref(at)?;
                    Place::Elements(start + 4, buf.u32(start)? as usize)
                }
                Kind::Array(len) => Place::Elements(at, len),
            };
        }
        Ok(Found::Here(place))
    }

    /// The value at the end of the path.
    fn read<'a>(&self, data: &'a [u8]) -> Result<Value<'a>, EvalError> {
        match self.walk(data)? {
            Found::Here(Place::Value(value)) | Found::Default(value) => Ok(value),
            Found::Absent(i) => Err(EvalError(format!(
                "{} isn't set",
                &self.text[..self.ends[i]]
            ))),
            Found::PastEnd(i, len) => Err(EvalError(format!(
                "{} is past the end: there are {len}",
                &self.text[..self.ends[i]]
            ))),
            Found::Here(_) => unreachable!("Paths read are checked to end in a value"),
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Types and compiled expressions.

#[derive(Clone, Copy, Debug, PartialEq)]
enum Ty {
    Bool,
    Int,
    Float,
    Str,
    /// An enum, by its index in the schema; an integer to arithmetic.
    Enum(usize),
    /// Only stale() takes one.
    Duration,
    /// A table, struct or vector: only for has() and size().
    Composite {
        vector: bool,
    },
}

impl Ty {
    fn numeric(self) -> bool {
        matches!(self, Ty::Int | Ty::Float | Ty::Enum(_))
    }

    fn integer(self) -> bool {
        matches!(self, Ty::Int | Ty::Enum(_))
    }
}

#[derive(Clone, Copy, Debug)]
enum Arith {
    Add,
    Sub,
    Mul,
    Div,
    Rem,
}

#[derive(Clone, Copy, Debug)]
enum Cmp {
    Eq,
    Ne,
    Lt,
    Le,
    Gt,
    Ge,
}

#[derive(Clone, Debug)]
enum Const {
    Bool(bool),
    Int(i128),
    Float(f64),
    Str(String),
}

#[derive(Clone, Debug)]
enum Expr {
    Const(Const),
    Read(Path),
    Has(Path),
    Size(Path),
    Changed(Path),
    Delta(Path),
    Rate(Path),
    Age,
    Stale(i64),
    Not(Box<Expr>),
    Neg(Box<Expr>),
    ToFloat(Box<Expr>),
    Arith(Arith, Box<Expr>, Box<Expr>),
    Cmp(Cmp, Box<Expr>, Box<Expr>),
    And(Box<Expr>, Box<Expr>),
    Or(Box<Expr>, Box<Expr>),
    If(Box<Expr>, Box<Expr>, Box<Expr>),
    Math1(fn(f64) -> f64, Box<Expr>),
    Math2(fn(f64, f64) -> f64, Box<Expr>, Box<Expr>),
    Test(fn(f64) -> bool, Box<Expr>),
    AbsInt(Box<Expr>),
    MinMax { max: bool, args: Vec<Expr> },
    Clamp(Box<Expr>, Box<Expr>, Box<Expr>),
    Near(Box<Expr>, Box<Expr>, Box<Expr>),
}

/// A function's kind: what it takes and gives.
#[derive(Clone, Copy)]
enum Func {
    Math1(fn(f64) -> f64),
    Math2(fn(f64, f64) -> f64),
    Test(fn(f64) -> bool),
    Abs,
    Min,
    Max,
    Clamp,
    Near,
    Has,
    Size,
    Changed,
    Delta,
    Rate,
    Age,
    Stale,
}

/// Wraps an angle to (-π, π].
fn wrap(angle: f64) -> f64 {
    use std::f64::consts::PI;
    angle - 2.0 * PI * ((angle - PI) / (2.0 * PI)).ceil()
}

fn sign(x: f64) -> f64 {
    if x == 0.0 || x.is_nan() {
        x
    } else {
        x.signum()
    }
}

/// Every function, for looking them up and suggesting them.
const FUNCTIONS: &[(&str, Func)] = &[
    ("abs", Func::Abs),
    ("sqrt", Func::Math1(f64::sqrt)),
    ("exp", Func::Math1(f64::exp)),
    ("log", Func::Math1(f64::ln)),
    ("log10", Func::Math1(f64::log10)),
    ("sign", Func::Math1(sign)),
    ("pow", Func::Math2(f64::powf)),
    ("hypot", Func::Math2(f64::hypot)),
    ("sin", Func::Math1(f64::sin)),
    ("cos", Func::Math1(f64::cos)),
    ("tan", Func::Math1(f64::tan)),
    ("asin", Func::Math1(f64::asin)),
    ("acos", Func::Math1(f64::acos)),
    ("atan", Func::Math1(f64::atan)),
    ("atan2", Func::Math2(f64::atan2)),
    ("degrees", Func::Math1(f64::to_degrees)),
    ("radians", Func::Math1(f64::to_radians)),
    ("wrap", Func::Math1(wrap)),
    ("floor", Func::Math1(f64::floor)),
    ("ceil", Func::Math1(f64::ceil)),
    ("round", Func::Math1(f64::round)),
    ("trunc", Func::Math1(f64::trunc)),
    ("min", Func::Min),
    ("max", Func::Max),
    ("clamp", Func::Clamp),
    ("near", Func::Near),
    ("isnan", Func::Test(f64::is_nan)),
    ("isinf", Func::Test(f64::is_infinite)),
    ("isfinite", Func::Test(f64::is_finite)),
    ("has", Func::Has),
    ("size", Func::Size),
    ("changed", Func::Changed),
    ("delta", Func::Delta),
    ("rate", Func::Rate),
    ("age", Func::Age),
    ("stale", Func::Stale),
];

/// The names of every function, for the tool's description.
pub fn function_names() -> impl Iterator<Item = &'static str> {
    FUNCTIONS.iter().map(|(name, _)| *name)
}

/// How many single-character edits apart `a` and `b` are.
fn edit_distance(a: &str, b: &str) -> usize {
    let b: Vec<char> = b.chars().collect();
    let mut row: Vec<usize> = (0..=b.len()).collect();
    for (i, ca) in a.chars().enumerate() {
        let mut previous = row[0];
        row[0] = i + 1;
        for (j, cb) in b.iter().enumerate() {
            let substitute = previous + usize::from(ca != *cb);
            previous = row[j + 1];
            row[j + 1] = substitute.min(row[j] + 1).min(row[j + 1] + 1);
        }
    }
    row[b.len()]
}

/// " Did you mean ...?" with the closest of `candidates` to `name`, or nothing.
fn suggest<'a>(name: &str, candidates: impl Iterator<Item = &'a str>) -> String {
    let mut scored: Vec<(usize, &str)> = candidates
        .map(|candidate| (edit_distance(name, candidate), candidate))
        .filter(|(distance, candidate)| {
            *distance <= (name.len().max(candidate.len()) / 3).max(2)
                || candidate.contains(name)
                || name.contains(candidate)
        })
        .collect();
    scored.sort();
    let names: Vec<&str> = scored.iter().take(3).map(|(_, name)| *name).collect();
    if names.is_empty() {
        String::new()
    } else {
        let (last, rest) = names.split_last().expect("Not empty");
        if rest.is_empty() {
            format!(" Did you mean {last}?")
        } else {
            format!(" Did you mean {} or {last}?", rest.join(", "))
        }
    }
}

fn schema_object<'a>(schema: &Schema<'a>, index: i32) -> Option<Object<'a>> {
    let objects = schema.objects();
    usize::try_from(index)
        .ok()
        .filter(|&i| i < objects.len())
        .map(|i| objects.get(i))
}

fn integer_base(base: BaseType) -> bool {
    matches!(
        base,
        BaseType::Byte
            | BaseType::UByte
            | BaseType::Short
            | BaseType::UShort
            | BaseType::Int
            | BaseType::UInt
            | BaseType::Long
            | BaseType::ULong
    )
}

fn scalar_ty(base: BaseType, enum_index: i32) -> Ty {
    match base {
        BaseType::Bool => Ty::Bool,
        BaseType::Float | BaseType::Double => Ty::Float,
        _ if enum_index >= 0 => Ty::Enum(enum_index as usize),
        _ => Ty::Int,
    }
}

fn scalar_size(base: BaseType) -> usize {
    match base {
        BaseType::Bool | BaseType::Byte | BaseType::UByte | BaseType::UType => 1,
        BaseType::Short | BaseType::UShort => 2,
        BaseType::Int | BaseType::UInt | BaseType::Float => 4,
        BaseType::Long | BaseType::ULong | BaseType::Double => 8,
        _ => 4,
    }
}

/// What a vector's or array's elements are.
#[derive(Clone, Copy)]
struct Elements {
    base: BaseType,
    index: i32,
    size: usize,
}

struct Compiler<'s, 'a> {
    schema: &'s Schema<'a>,
    root: Object<'a>,
    source: &'s str,
}

impl<'a> Compiler<'_, 'a> {
    fn enum_name(&self, index: usize) -> String {
        self.schema.enums().get(index).name().to_string()
    }

    /// The value of enum `index`'s value `name`.
    fn enum_value(&self, index: usize, name: &str, span: Span) -> Result<i128, CompileError> {
        let values = self.schema.enums().get(index).values();
        if let Some(value) = values.iter().find(|value| value.name() == name) {
            return Ok(value.value() as i128);
        }
        let names: Vec<&str> = values.iter().map(|value| value.name()).collect();
        Err(CompileError::new(
            span,
            format!(
                "{} has no value {name:?}: it's one of {}",
                self.enum_name(index),
                names.join(", ")
            ),
        ))
    }

    fn path(&self, segs: &[Seg]) -> Result<(Path, Ty), CompileError> {
        enum At<'a> {
            Object(Object<'a>),
            Elements(Elements),
            Value(Ty),
        }
        let first = match &segs[0] {
            Seg::Field(_, span) => span.0,
            Seg::Index(_, span) => span.0,
        };
        let mut at = At::Object(self.root);
        let mut steps = Vec::new();
        let mut ends = Vec::new();
        for seg in segs {
            let (span, step, next) = match (seg, &at) {
                (Seg::Field(name, span), At::Object(object)) => {
                    let fields = object.fields();
                    let Some(field) = fields
                        .iter()
                        .find(|field| field.name() == name && !field.deprecated())
                    else {
                        let names: Vec<&str> = fields
                            .iter()
                            .filter(|field| !field.deprecated())
                            .map(|field| field.name())
                            .collect();
                        let mut message = format!("{} has no field {name}.", object.name());
                        let suggestion = suggest(name, names.iter().copied());
                        if suggestion.is_empty() {
                            message.push_str(&format!(" Its fields are {}.", names.join(", ")));
                        } else {
                            message.push_str(&suggestion);
                        }
                        return Err(CompileError::new(*span, message));
                    };
                    let type_ = field.type_();
                    let base = type_.base_type();
                    let (kind, next) = match base {
                        BaseType::Bool
                        | BaseType::Byte
                        | BaseType::UByte
                        | BaseType::Short
                        | BaseType::UShort
                        | BaseType::Int
                        | BaseType::UInt
                        | BaseType::Long
                        | BaseType::ULong
                        | BaseType::Float
                        | BaseType::Double => {
                            let default = if field.optional() {
                                None
                            } else if base == BaseType::Bool {
                                Some(DefaultValue::Bool(field.default_integer() != 0))
                            } else if integer_base(base) {
                                Some(DefaultValue::Int(field.default_integer()))
                            } else {
                                Some(DefaultValue::Float(field.default_real()))
                            };
                            (
                                Kind::Scalar(base, default),
                                At::Value(scalar_ty(base, type_.index())),
                            )
                        }
                        BaseType::String => (Kind::String, At::Value(Ty::Str)),
                        BaseType::Obj => {
                            let child = schema_object(self.schema, type_.index())
                                .ok_or_else(|| CompileError::new(*span, "Not in the schema"))?;
                            let kind = if child.is_struct() {
                                Kind::Struct
                            } else {
                                Kind::Table
                            };
                            (kind, At::Object(child))
                        }
                        BaseType::Vector | BaseType::Array => {
                            let element = type_.element();
                            let mut size = type_.element_size() as usize;
                            if size == 0 {
                                size = match element {
                                    BaseType::Obj => schema_object(self.schema, type_.index())
                                        .filter(Object::is_struct)
                                        .map_or(4, |object| object.bytesize() as usize),
                                    _ => scalar_size(element),
                                };
                            }
                            if matches!(element, BaseType::Union | BaseType::UType) {
                                return Err(CompileError::new(
                                    *span,
                                    format!("{name} is a vector of unions, which conditions can't read yet"),
                                ));
                            }
                            let kind = if base == BaseType::Array {
                                Kind::Array(type_.fixed_length() as usize)
                            } else {
                                Kind::Vector
                            };
                            (
                                kind,
                                At::Elements(Elements {
                                    base: element,
                                    index: type_.index(),
                                    size,
                                }),
                            )
                        }
                        _ => {
                            return Err(CompileError::new(
                                *span,
                                format!("{name} is a union, which conditions can't read yet"),
                            ))
                        }
                    };
                    let step = if object.is_struct() {
                        Step::StructField {
                            offset: field.offset(),
                            kind,
                        }
                    } else {
                        Step::TableField {
                            slot: field.offset(),
                            kind,
                        }
                    };
                    (*span, step, next)
                }
                (Seg::Index(index, span), At::Elements(elements)) => {
                    let (kind, next) = match elements.base {
                        BaseType::String => (Kind::String, At::Value(Ty::Str)),
                        BaseType::Obj => {
                            let child = schema_object(self.schema, elements.index)
                                .ok_or_else(|| CompileError::new(*span, "Not in the schema"))?;
                            let kind = if child.is_struct() {
                                Kind::Struct
                            } else {
                                Kind::Table
                            };
                            (kind, At::Object(child))
                        }
                        base => (
                            Kind::Scalar(base, None),
                            At::Value(scalar_ty(base, elements.index)),
                        ),
                    };
                    let step = Step::Index {
                        index: *index,
                        size: elements.size,
                        kind,
                    };
                    (*span, step, next)
                }
                (Seg::Field(name, span), At::Elements(_)) => {
                    return Err(CompileError::new(
                        *span,
                        format!("This is a vector: index it, like [0], before .{name}"),
                    ))
                }
                (Seg::Field(name, span), At::Value(_)) => {
                    return Err(CompileError::new(
                        *span,
                        format!("This has no fields, so no .{name}"),
                    ))
                }
                (Seg::Index(_, span), _) => {
                    return Err(CompileError::new(*span, "Only vectors can be indexed"))
                }
            };
            steps.push(step);
            ends.push(span.1 - first);
            at = next;
        }
        let last = match segs.last().expect("A path has a segment") {
            Seg::Field(_, span) | Seg::Index(_, span) => span.1,
        };
        let ty = match at {
            At::Value(ty) => ty,
            At::Object(_) => Ty::Composite { vector: false },
            At::Elements(_) => Ty::Composite { vector: true },
        };
        Ok((
            Path {
                text: self.source[first..last].to_string(),
                ends,
                steps,
            },
            ty,
        ))
    }

    fn describe(ty: Ty) -> &'static str {
        match ty {
            Ty::Bool => "a bool",
            Ty::Int => "an integer",
            Ty::Float => "a number",
            Ty::Str => "a string",
            Ty::Enum(_) => "an enum",
            Ty::Duration => "a duration",
            Ty::Composite { vector: true } => "a vector",
            Ty::Composite { vector: false } => "a table or struct",
        }
    }

    /// Compiles `node`, which must be a value.
    fn value(&self, node: &Node) -> Result<(Expr, Ty), CompileError> {
        let (expr, ty) = self.expr(node)?;
        match ty {
            Ty::Composite { .. } => Err(CompileError::new(
                node.span,
                format!(
                    "This is {}, not a value: pick a field of it, or use has() or size()",
                    Self::describe(ty)
                ),
            )),
            Ty::Duration => Err(CompileError::new(
                node.span,
                "Durations only go in stale(), like stale(500ms)",
            )),
            _ => Ok((expr, ty)),
        }
    }

    fn numeric(&self, node: &Node) -> Result<(Expr, Ty), CompileError> {
        let (expr, ty) = self.value(node)?;
        if !ty.numeric() {
            return Err(CompileError::new(
                node.span,
                format!("This is {}, where a number goes", Self::describe(ty)),
            ));
        }
        Ok((expr, ty))
    }

    fn boolean(&self, node: &Node) -> Result<Expr, CompileError> {
        let (expr, ty) = self.value(node)?;
        if ty != Ty::Bool {
            return Err(CompileError::new(
                node.span,
                format!("This is {}, where a bool goes", Self::describe(ty)),
            ));
        }
        Ok(expr)
    }

    fn path_arg(&self, name: &str, args: &[Node]) -> Result<(Path, Ty), CompileError> {
        match &args[0].ast {
            Ast::Path(segs) => self.path(segs),
            _ => Err(CompileError::new(
                args[0].span,
                format!("{name}() takes a field, like {name}(left.position)"),
            )),
        }
    }

    fn call(
        &self,
        name: &str,
        name_span: Span,
        span: Span,
        args: &[Node],
    ) -> Result<(Expr, Ty), CompileError> {
        let Some((_, func)) = FUNCTIONS.iter().find(|(n, _)| *n == name) else {
            return Err(CompileError::new(
                name_span,
                format!(
                    "There's no function {name}.{}",
                    suggest(name, function_names())
                ),
            ));
        };
        let (min, max) = match func {
            Func::Math1(_) | Func::Test(_) | Func::Abs => (1, 1),
            Func::Math2(_) => (2, 2),
            Func::Min | Func::Max => (2, usize::MAX),
            Func::Clamp | Func::Near => (3, 3),
            Func::Has | Func::Size | Func::Changed | Func::Delta | Func::Rate | Func::Stale => {
                (1, 1)
            }
            Func::Age => (0, 0),
        };
        if args.len() < min || args.len() > max {
            let wanted = if min == max {
                format!("{min}")
            } else {
                format!("at least {min}")
            };
            return Err(CompileError::new(
                span,
                format!("{name}() takes {wanted} arguments, not {}", args.len()),
            ));
        }
        let float = |node: &Node| -> Result<Expr, CompileError> {
            let (expr, ty) = self.numeric(node)?;
            Ok(if ty == Ty::Float {
                expr
            } else {
                Expr::ToFloat(Box::new(expr))
            })
        };
        Ok(match *func {
            Func::Math1(f) => (Expr::Math1(f, Box::new(float(&args[0])?)), Ty::Float),
            Func::Math2(f) => (
                Expr::Math2(f, Box::new(float(&args[0])?), Box::new(float(&args[1])?)),
                Ty::Float,
            ),
            Func::Test(f) => (Expr::Test(f, Box::new(float(&args[0])?)), Ty::Bool),
            Func::Abs => {
                let (expr, ty) = self.numeric(&args[0])?;
                if ty.integer() {
                    (Expr::AbsInt(Box::new(expr)), Ty::Int)
                } else {
                    (Expr::Math1(f64::abs, Box::new(expr)), Ty::Float)
                }
            }
            Func::Min | Func::Max => {
                let mut exprs = Vec::new();
                let mut all_integers = true;
                for arg in args {
                    let (expr, ty) = self.numeric(arg)?;
                    all_integers &= ty.integer();
                    exprs.push(expr);
                }
                (
                    Expr::MinMax {
                        max: matches!(func, Func::Max),
                        args: exprs,
                    },
                    if all_integers { Ty::Int } else { Ty::Float },
                )
            }
            Func::Clamp => (
                Expr::Clamp(
                    Box::new(float(&args[0])?),
                    Box::new(float(&args[1])?),
                    Box::new(float(&args[2])?),
                ),
                Ty::Float,
            ),
            Func::Near => (
                Expr::Near(
                    Box::new(float(&args[0])?),
                    Box::new(float(&args[1])?),
                    Box::new(float(&args[2])?),
                ),
                Ty::Bool,
            ),
            Func::Has => (Expr::Has(self.path_arg(name, args)?.0), Ty::Bool),
            Func::Size => {
                let (path, ty) = self.path_arg(name, args)?;
                if !matches!(ty, Ty::Composite { vector: true } | Ty::Str) {
                    return Err(CompileError::new(
                        args[0].span,
                        format!(
                            "This is {}: size() takes a vector or a string",
                            Self::describe(ty)
                        ),
                    ));
                }
                (Expr::Size(path), Ty::Int)
            }
            Func::Changed | Func::Delta | Func::Rate => {
                let (path, ty) = self.path_arg(name, args)?;
                if matches!(ty, Ty::Composite { .. }) {
                    return Err(CompileError::new(
                        args[0].span,
                        format!("This is {}: pick a field of it", Self::describe(ty)),
                    ));
                }
                if !matches!(func, Func::Changed) && !ty.numeric() {
                    return Err(CompileError::new(
                        args[0].span,
                        format!("This is {}: {name}() takes a number", Self::describe(ty)),
                    ));
                }
                match func {
                    Func::Changed => (Expr::Changed(path), Ty::Bool),
                    Func::Delta => (Expr::Delta(path), Ty::Float),
                    _ => (Expr::Rate(path), Ty::Float),
                }
            }
            Func::Age => (Expr::Age, Ty::Float),
            Func::Stale => match args[0].ast {
                Ast::Duration(ns) => (Expr::Stale(ns), Ty::Bool),
                _ => {
                    return Err(CompileError::new(
                        args[0].span,
                        "stale() takes a duration, like stale(500ms)",
                    ))
                }
            },
        })
    }

    /// `node` as an enum's value, if it's a string and the other side is enum `ty`.
    fn enum_literal(&self, node: &Node, ty: Ty) -> Result<Option<Expr>, CompileError> {
        match (&node.ast, ty) {
            (Ast::Str(name), Ty::Enum(index)) => Ok(Some(Expr::Const(Const::Int(
                self.enum_value(index, name, node.span)?,
            )))),
            _ => Ok(None),
        }
    }

    fn compare(&self, op: &str, lhs: &Node, rhs: &Node, span: Span) -> Result<Expr, CompileError> {
        let cmp = match op {
            "==" => Cmp::Eq,
            "!=" => Cmp::Ne,
            "<" => Cmp::Lt,
            "<=" => Cmp::Le,
            ">" => Cmp::Gt,
            _ => Cmp::Ge,
        };
        let equality = matches!(cmp, Cmp::Eq | Cmp::Ne);
        // An enum against a name: compare its value with the name's.
        let (a, ta, b, tb) = if let Ast::Str(_) = lhs.ast {
            let (b, tb) = self.value(rhs)?;
            match self.enum_literal(lhs, tb)? {
                Some(a) => (a, tb, b, tb),
                None => {
                    let (a, ta) = self.value(lhs)?;
                    (a, ta, b, tb)
                }
            }
        } else {
            let (a, ta) = self.value(lhs)?;
            match self.enum_literal(rhs, ta)? {
                Some(b) => (a, ta, b, ta),
                None => {
                    let (b, tb) = self.value(rhs)?;
                    (a, ta, b, tb)
                }
            }
        };
        let ok = match (ta, tb) {
            (Ty::Enum(x), Ty::Enum(y)) if x != y => {
                return Err(CompileError::new(
                    span,
                    format!(
                        "These are different enums, {} and {}",
                        self.enum_name(x),
                        self.enum_name(y)
                    ),
                ))
            }
            (x, y) if x.numeric() && y.numeric() => true,
            (Ty::Bool, Ty::Bool) | (Ty::Str, Ty::Str) => equality,
            _ => false,
        };
        if !ok {
            let why = if ta == tb {
                format!("{} can only be compared with == and !=", Self::describe(ta))
            } else {
                format!(
                    "This compares {} with {}",
                    Self::describe(ta),
                    Self::describe(tb)
                )
            };
            return Err(CompileError::new(span, why));
        }
        Ok(Expr::Cmp(cmp, Box::new(a), Box::new(b)))
    }

    fn expr(&self, node: &Node) -> Result<(Expr, Ty), CompileError> {
        Ok(match &node.ast {
            Ast::Int(v) => (Expr::Const(Const::Int(*v)), Ty::Int),
            Ast::Float(v) => (Expr::Const(Const::Float(*v)), Ty::Float),
            Ast::Bool(v) => (Expr::Const(Const::Bool(*v)), Ty::Bool),
            Ast::Str(v) => (Expr::Const(Const::Str(v.clone())), Ty::Str),
            Ast::Duration(_) => (Expr::Const(Const::Int(0)), Ty::Duration),
            Ast::Path(segs) => {
                // pi, inf and nan, unless the message has a field by that name.
                if let [Seg::Field(name, _)] = segs.as_slice() {
                    let constant = match name.as_str() {
                        "pi" => Some(std::f64::consts::PI),
                        "inf" => Some(f64::INFINITY),
                        "nan" => Some(f64::NAN),
                        _ => None,
                    };
                    if let Some(constant) = constant {
                        if !self.root.fields().iter().any(|field| field.name() == name) {
                            return Ok((Expr::Const(Const::Float(constant)), Ty::Float));
                        }
                    }
                }
                let (path, ty) = self.path(segs)?;
                (Expr::Read(path), ty)
            }
            Ast::Call(name, name_span, args) => self.call(name, *name_span, node.span, args)?,
            Ast::Unary(op, operand) => {
                if *op == "!" {
                    (Expr::Not(Box::new(self.boolean(operand)?)), Ty::Bool)
                } else {
                    let (expr, ty) = self.numeric(operand)?;
                    let ty = if ty.integer() { Ty::Int } else { Ty::Float };
                    (Expr::Neg(Box::new(expr)), ty)
                }
            }
            Ast::Binary(op, lhs, rhs) => match *op {
                "&&" | "||" => {
                    let (a, b) = (self.boolean(lhs)?, self.boolean(rhs)?);
                    let expr = if *op == "&&" {
                        Expr::And(Box::new(a), Box::new(b))
                    } else {
                        Expr::Or(Box::new(a), Box::new(b))
                    };
                    (expr, Ty::Bool)
                }
                "==" | "!=" | "<" | "<=" | ">" | ">=" => {
                    (self.compare(op, lhs, rhs, node.span)?, Ty::Bool)
                }
                _ => {
                    let (a, ta) = self.numeric(lhs)?;
                    let (b, tb) = self.numeric(rhs)?;
                    let arith = match *op {
                        "+" => Arith::Add,
                        "-" => Arith::Sub,
                        "*" => Arith::Mul,
                        "/" => Arith::Div,
                        _ => Arith::Rem,
                    };
                    let ty = if matches!(arith, Arith::Div) {
                        Ty::Float
                    } else if ta.integer() && tb.integer() {
                        Ty::Int
                    } else {
                        Ty::Float
                    };
                    (Expr::Arith(arith, Box::new(a), Box::new(b)), ty)
                }
            },
            Ast::Ternary(condition, yes, no) => {
                let condition = self.boolean(condition)?;
                let (a, ta) = self.value(yes)?;
                let (b, tb) = self.value(no)?;
                let (a, b, ty) = if ta == tb {
                    (a, b, ta)
                } else if ta.integer() && tb.integer() {
                    (a, b, Ty::Int)
                } else if ta.numeric() && tb.numeric() {
                    let float = |expr, ty| {
                        if ty == Ty::Float {
                            expr
                        } else {
                            Expr::ToFloat(Box::new(expr))
                        }
                    };
                    (float(a, ta), float(b, tb), Ty::Float)
                } else {
                    return Err(CompileError::new(
                        (yes.span.0, no.span.1),
                        format!(
                            "?:'s choices must be the same type, not {} and {}",
                            Self::describe(ta),
                            Self::describe(tb)
                        ),
                    ));
                };
                (Expr::If(Box::new(condition), Box::new(a), Box::new(b)), ty)
            }
        })
    }
}

// ---------------------------------------------------------------------------------------------
// Evaluating.

/// One message on the channel, and when it was sent.
#[derive(Clone, Copy, Debug)]
pub struct Message<'a> {
    pub data: &'a [u8],
    pub monotonic_ns: i64,
}

/// What a condition is evaluated on.
#[derive(Clone, Copy, Debug)]
pub struct Input<'a> {
    /// The message, or None if the channel has had none.
    pub message: Option<Message<'a>>,
    /// The message before it, for changed(), delta() and rate().
    pub previous: Option<Message<'a>>,
    /// Now, on the event loop's monotonic clock.
    pub now_ns: i64,
    /// When the channel last got a message, or when waiting started if it hasn't: what
    /// stale() measures from.
    pub last_ns: i64,
}

fn arith(op: Arith, a: Value, b: Value) -> Result<Value<'static>, EvalError> {
    if let (Value::Int(x), Value::Int(y), false) = (a, b, matches!(op, Arith::Div)) {
        let result = match op {
            Arith::Add => x.checked_add(y),
            Arith::Sub => x.checked_sub(y),
            Arith::Mul => x.checked_mul(y),
            Arith::Rem if y == 0 => return Err(EvalError("% by zero".into())),
            _ => x.checked_rem(y),
        };
        return result
            .map(Value::Int)
            .ok_or_else(|| EvalError("Integer overflow".into()));
    }
    let (x, y) = (a.as_f64(), b.as_f64());
    Ok(Value::Float(match op {
        Arith::Add => x + y,
        Arith::Sub => x - y,
        Arith::Mul => x * y,
        Arith::Div => x / y,
        Arith::Rem => x % y,
    }))
}

fn compare(op: Cmp, a: Value, b: Value) -> bool {
    use std::cmp::Ordering;
    let ordering = match (a, b) {
        (Value::Int(x), Value::Int(y)) => Some(x.cmp(&y)),
        (Value::Bool(x), Value::Bool(y)) => Some(x.cmp(&y)),
        (Value::Str(x), Value::Str(y)) => Some(x.cmp(y)),
        _ => a.as_f64().partial_cmp(&b.as_f64()),
    };
    match (op, ordering) {
        (Cmp::Ne, None) => true,
        (_, None) => false,
        (Cmp::Eq, Some(o)) => o == Ordering::Equal,
        (Cmp::Ne, Some(o)) => o != Ordering::Equal,
        (Cmp::Lt, Some(o)) => o == Ordering::Less,
        (Cmp::Le, Some(o)) => o != Ordering::Greater,
        (Cmp::Gt, Some(o)) => o == Ordering::Greater,
        (Cmp::Ge, Some(o)) => o != Ordering::Less,
    }
}

fn need<'a>(message: Option<Message<'a>>) -> Result<Message<'a>, EvalError> {
    message.ok_or_else(|| EvalError("Nothing has been sent on the channel yet".into()))
}

impl Expr {
    fn eval<'a>(&'a self, input: &Input<'a>) -> Result<Value<'a>, EvalError> {
        Ok(match self {
            Expr::Const(c) => match c {
                Const::Bool(v) => Value::Bool(*v),
                Const::Int(v) => Value::Int(*v),
                Const::Float(v) => Value::Float(*v),
                Const::Str(v) => Value::Str(v.as_str()),
            },
            Expr::Read(path) => path.read(need(input.message)?.data)?,
            Expr::Has(path) => Value::Bool(matches!(
                path.walk(need(input.message)?.data)?,
                Found::Here(_)
            )),
            Expr::Size(path) => match path.walk(need(input.message)?.data)? {
                Found::Here(Place::Elements(_, len)) => Value::Int(len as i128),
                Found::Here(Place::Value(Value::Str(s))) => Value::Int(s.len() as i128),
                Found::PastEnd(..) => {
                    return Err(path.read(need(input.message)?.data).unwrap_err())
                }
                _ => Value::Int(0),
            },
            Expr::Changed(path) => {
                let current = path.read(need(input.message)?.data)?;
                match input.previous {
                    None => Value::Bool(false),
                    Some(previous) => {
                        let before = path.read(previous.data)?;
                        Value::Bool(match (current, before) {
                            (Value::Float(a), Value::Float(b)) => a.to_bits() != b.to_bits(),
                            (a, b) => a != b,
                        })
                    }
                }
            }
            Expr::Delta(path) | Expr::Rate(path) => {
                let message = need(input.message)?;
                let Some(previous) = input.previous else {
                    return Ok(Value::Float(f64::NAN));
                };
                let delta = match (path.read(message.data)?, path.read(previous.data)?) {
                    (Value::Int(a), Value::Int(b)) => (a - b) as f64,
                    (a, b) => a.as_f64() - b.as_f64(),
                };
                if let Expr::Rate(_) = self {
                    let dt = (message.monotonic_ns - previous.monotonic_ns) as f64 / 1e9;
                    Value::Float(if dt == 0.0 { f64::NAN } else { delta / dt })
                } else {
                    Value::Float(delta)
                }
            }
            Expr::Age => {
                let message = need(input.message)?;
                Value::Float((input.now_ns - message.monotonic_ns) as f64 / 1e9)
            }
            Expr::Stale(ns) => Value::Bool(input.now_ns - input.last_ns > *ns),
            Expr::Not(e) => Value::Bool(!matches!(e.eval(input)?, Value::Bool(true))),
            Expr::Neg(e) => match e.eval(input)? {
                Value::Int(v) => Value::Int(
                    v.checked_neg()
                        .ok_or_else(|| EvalError("Integer overflow".into()))?,
                ),
                v => Value::Float(-v.as_f64()),
            },
            Expr::ToFloat(e) => Value::Float(e.eval(input)?.as_f64()),
            Expr::Arith(op, a, b) => arith(*op, a.eval(input)?, b.eval(input)?)?,
            Expr::Cmp(op, a, b) => Value::Bool(compare(*op, a.eval(input)?, b.eval(input)?)),
            Expr::And(a, b) => Value::Bool(
                a.eval(input)? == Value::Bool(true) && b.eval(input)? == Value::Bool(true),
            ),
            Expr::Or(a, b) => Value::Bool(
                a.eval(input)? == Value::Bool(true) || b.eval(input)? == Value::Bool(true),
            ),
            Expr::If(c, a, b) => {
                if c.eval(input)? == Value::Bool(true) {
                    a.eval(input)?
                } else {
                    b.eval(input)?
                }
            }
            Expr::Math1(f, a) => Value::Float(f(a.eval(input)?.as_f64())),
            Expr::Math2(f, a, b) => {
                Value::Float(f(a.eval(input)?.as_f64(), b.eval(input)?.as_f64()))
            }
            Expr::Test(f, a) => Value::Bool(f(a.eval(input)?.as_f64())),
            Expr::AbsInt(a) => match a.eval(input)? {
                Value::Int(v) => Value::Int(v.abs()),
                v => Value::Float(v.as_f64().abs()),
            },
            Expr::MinMax { max, args } => {
                let mut best = args[0].eval(input)?;
                for arg in &args[1..] {
                    let value = arg.eval(input)?;
                    let better = if *max {
                        compare(Cmp::Gt, value, best)
                    } else {
                        compare(Cmp::Lt, value, best)
                    };
                    if better || best.as_f64().is_nan() {
                        best = value;
                    }
                }
                best
            }
            Expr::Clamp(x, lo, hi) => {
                let (x, lo, hi) = (
                    x.eval(input)?.as_f64(),
                    lo.eval(input)?.as_f64(),
                    hi.eval(input)?.as_f64(),
                );
                Value::Float(x.max(lo).min(hi))
            }
            Expr::Near(a, b, tolerance) => Value::Bool(
                (a.eval(input)?.as_f64() - b.eval(input)?.as_f64()).abs()
                    <= tolerance.eval(input)?.as_f64(),
            ),
        })
    }

    /// Every field the expression reads as a value, for summarizing what it saw.
    fn reads<'e>(&'e self, out: &mut Vec<&'e Path>) {
        match self {
            Expr::Read(path) | Expr::Changed(path) | Expr::Delta(path) | Expr::Rate(path) => {
                if !out.iter().any(|p| p.text == path.text) {
                    out.push(path);
                }
            }
            Expr::Not(a) | Expr::Neg(a) | Expr::ToFloat(a) | Expr::Math1(_, a) => a.reads(out),
            Expr::Test(_, a) | Expr::AbsInt(a) => a.reads(out),
            Expr::Arith(_, a, b) | Expr::Cmp(_, a, b) | Expr::And(a, b) | Expr::Or(a, b) => {
                a.reads(out);
                b.reads(out);
            }
            Expr::Math2(_, a, b) => {
                a.reads(out);
                b.reads(out);
            }
            Expr::If(a, b, c) | Expr::Clamp(a, b, c) | Expr::Near(a, b, c) => {
                a.reads(out);
                b.reads(out);
                c.reads(out);
            }
            Expr::MinMax { args, .. } => args.iter().for_each(|a| a.reads(out)),
            Expr::Const(_) | Expr::Has(_) | Expr::Size(_) | Expr::Age | Expr::Stale(_) => {}
        }
    }

    /// Whether the expression depends on the time, not only on messages.
    fn timed(&self) -> bool {
        match self {
            Expr::Age | Expr::Stale(_) => true,
            Expr::Not(a) | Expr::Neg(a) | Expr::ToFloat(a) | Expr::Math1(_, a) => a.timed(),
            Expr::Test(_, a) | Expr::AbsInt(a) => a.timed(),
            Expr::Arith(_, a, b)
            | Expr::Cmp(_, a, b)
            | Expr::And(a, b)
            | Expr::Or(a, b)
            | Expr::Math2(_, a, b) => a.timed() || b.timed(),
            Expr::If(a, b, c) | Expr::Clamp(a, b, c) | Expr::Near(a, b, c) => {
                a.timed() || b.timed() || c.timed()
            }
            Expr::MinMax { args, .. } => args.iter().any(Expr::timed),
            _ => false,
        }
    }
}

/// A compiled condition.
#[derive(Clone, Debug)]
pub struct Condition {
    source: String,
    expr: Expr,
    timed: bool,
}

impl Condition {
    /// Compiles `source` for messages of `schema`'s root table.
    pub fn compile(source: &str, schema: &Schema) -> Result<Self, CompileError> {
        let root = schema.root_table().ok_or_else(|| {
            CompileError::new((0, source.len()), "The channel's schema has no root table")
        })?;
        let node = Parser {
            tokens: lex(source)?,
            pos: 0,
        }
        .whole()?;
        let compiler = Compiler {
            schema,
            root,
            source,
        };
        let (expr, ty) = compiler.value(&node)?;
        if ty != Ty::Bool {
            return Err(CompileError::new(
                node.span,
                format!(
                    "A condition is true or false, but this is {}",
                    Compiler::describe(ty)
                ),
            ));
        }
        let timed = expr.timed();
        Ok(Self {
            source: source.to_string(),
            expr,
            timed,
        })
    }

    pub fn source(&self) -> &str {
        &self.source
    }

    /// Whether it depends on the time (age() or stale()), and so can change between messages.
    pub fn timed(&self) -> bool {
        self.timed
    }

    pub fn eval(&self, input: &Input) -> Result<bool, EvalError> {
        Ok(self.expr.eval(input)? == Value::Bool(true))
    }

    /// The fields the condition reads, in the order it reads them.
    pub fn fields(&self) -> Vec<&str> {
        let mut paths = Vec::new();
        self.expr.reads(&mut paths);
        paths.into_iter().map(|path| path.text.as_str()).collect()
    }
}

/// What the fields a condition reads did, while it waited: for saying how close it came.
#[derive(Clone, Debug, PartialEq)]
pub struct FieldSummary {
    pub field: String,
    /// The last value read, as JSON would write it.
    pub last: Option<String>,
    /// For numbers, the smallest and largest seen.
    pub min: Option<f64>,
    pub max: Option<f64>,
}

/// Collects a [`FieldSummary`] for each field a condition reads.
#[derive(Clone, Debug)]
pub struct Summary {
    paths: Vec<Path>,
    fields: Vec<FieldSummary>,
}

impl Summary {
    pub fn new(condition: &Condition) -> Self {
        let mut paths = Vec::new();
        condition.expr.reads(&mut paths);
        let paths: Vec<Path> = paths.into_iter().cloned().collect();
        let fields = paths
            .iter()
            .map(|path| FieldSummary {
                field: path.text.clone(),
                last: None,
                min: None,
                max: None,
            })
            .collect();
        Self { paths, fields }
    }

    /// Takes in another message.
    pub fn add(&mut self, data: &[u8]) {
        for (path, field) in self.paths.iter().zip(&mut self.fields) {
            let Ok(value) = path.read(data) else {
                continue;
            };
            field.last = Some(value.to_string());
            if let Value::Int(_) | Value::Float(_) = value {
                let v = value.as_f64();
                if !v.is_nan() {
                    field.min = Some(field.min.map_or(v, |m| m.min(v)));
                    field.max = Some(field.max.map_or(v, |m| m.max(v)));
                }
            }
        }
    }

    pub fn fields(&self) -> &[FieldSummary] {
        &self.fields
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    pub(crate) use condition_test_rust_fbs::aos::robot_server::testing::State;
    use condition_test_rust_fbs::aos::robot_server::testing::{
        Segment, Status, StatusArgs, Vec2, Wheel, WheelArgs,
    };
    use flatbuffers::FlatBufferBuilder;
    use flatbuffers_reflection::reflection::root_as_schema;

    pub(crate) fn schema_bytes() -> Vec<u8> {
        std::fs::read(aos_testing_path::artifact_path(std::path::Path::new(
            "aos/network/robot_server/condition_test.bfbs",
        )))
        .unwrap()
    }

    /// What a test message holds; the rest of Status is left unset.
    #[derive(Default)]
    pub(crate) struct Fields {
        pub(crate) state: Option<State>,
        pub(crate) zeroed: bool,
        pub(crate) count: i32,
        pub(crate) big: u64,
        pub(crate) heading: f64,
        pub(crate) maybe: Option<i32>,
        pub(crate) label: Option<&'static str>,
        pub(crate) left: Option<(f64, f32)>,
        pub(crate) wheels: Vec<f64>,
        pub(crate) values: Vec<i32>,
        pub(crate) points: Vec<(f64, f64)>,
        pub(crate) states: Vec<State>,
        pub(crate) names: Vec<&'static str>,
        pub(crate) pose: Option<(f64, f64)>,
        pub(crate) segment: Option<[i32; 3]>,
    }

    pub(crate) fn message(fields: Fields) -> Vec<u8> {
        let mut fbb = FlatBufferBuilder::new();
        let label = fields.label.map(|s| fbb.create_string(s));
        let left = fields.left.map(|(position, velocity)| {
            let name = fbb.create_string("left");
            Wheel::create(
                &mut fbb,
                &WheelArgs {
                    position,
                    velocity,
                    name: Some(name),
                },
            )
        });
        let wheels: Vec<_> = fields
            .wheels
            .iter()
            .map(|&position| {
                Wheel::create(
                    &mut fbb,
                    &WheelArgs {
                        position,
                        ..Default::default()
                    },
                )
            })
            .collect();
        let wheels = (!wheels.is_empty()).then(|| fbb.create_vector(&wheels));
        let values = (!fields.values.is_empty()).then(|| fbb.create_vector(&fields.values));
        let points: Vec<Vec2> = fields
            .points
            .iter()
            .map(|&(x, y)| Vec2::new(x, y))
            .collect();
        let points = (!points.is_empty()).then(|| fbb.create_vector(&points));
        let states = (!fields.states.is_empty()).then(|| fbb.create_vector(&fields.states));
        let names: Vec<_> = fields.names.iter().map(|s| fbb.create_string(s)).collect();
        let names = (!names.is_empty()).then(|| fbb.create_vector(&names));
        let pose = fields.pose.map(|(x, y)| Vec2::new(x, y));
        let segment = fields
            .segment
            .map(|ids| Segment::new(&Vec2::new(1.0, 2.0), &Vec2::new(3.0, 4.0), &ids));
        let status = Status::create(
            &mut fbb,
            &StatusArgs {
                state: fields.state.unwrap_or(State::IDLE),
                zeroed: fields.zeroed,
                count: fields.count,
                big: fields.big,
                heading: fields.heading,
                scale: 1.5,
                maybe: fields.maybe,
                label,
                left,
                wheels,
                values,
                points,
                pose: pose.as_ref(),
                states,
                names,
                segment: segment.as_ref(),
            },
        );
        fbb.finish(status, None);
        fbb.finished_data().to_vec()
    }

    fn compile(source: &str) -> Result<Condition, String> {
        let bytes = schema_bytes();
        let schema = root_as_schema(&bytes).unwrap();
        Condition::compile(source, &schema).map_err(|error| error.render(source))
    }

    fn at(source: &str, data: &[u8], previous: Option<&[u8]>) -> Result<bool, EvalError> {
        let condition = compile(source).unwrap_or_else(|error| panic!("{error}"));
        condition.eval(&Input {
            message: Some(Message {
                data,
                monotonic_ns: 2_000_000_000,
            }),
            previous: previous.map(|data| Message {
                data,
                monotonic_ns: 1_500_000_000,
            }),
            now_ns: 2_250_000_000,
            last_ns: 2_000_000_000,
        })
    }

    fn holds(source: &str, data: &[u8]) -> bool {
        at(source, data, None).unwrap_or_else(|error| panic!("{source}: {error}"))
    }

    fn sample() -> Vec<u8> {
        message(Fields {
            state: Some(State::INTAKING),
            zeroed: true,
            count: 7,
            big: u64::MAX,
            heading: 3.0,
            label: Some("hello"),
            left: Some((1.25, -2.5)),
            wheels: vec![0.5, 1.5],
            values: vec![10, 20, 30],
            points: vec![(1.0, 2.0), (3.0, 4.0)],
            states: vec![State::SHOOTING],
            names: vec!["a", "bc"],
            pose: Some((5.0, 6.0)),
            segment: Some([7, 8, 9]),
            ..Default::default()
        })
    }

    #[test]
    fn reads_every_kind_of_field() {
        let data = sample();
        for source in [
            "zeroed",
            "count == 7",
            "big == 18446744073709551615",
            "heading > 2.9 && heading < 3.1",
            "scale == 1.5",
            "label == 'hello'",
            "left.position == 1.25",
            "left.velocity == -2.5",
            "left.name == \"left\"",
            "wheels[1].position == 1.5",
            "values[2] == 30",
            "points[1].y == 4",
            "pose.x == 5",
            "states[0] == 'SHOOTING'",
            "names[1] == 'bc'",
            "segment.end.x == 3 && segment.ids[2] == 9",
            "state == 'INTAKING'",
            "'INTAKING' == state",
            "state != 'IDLE' && state == 1",
        ] {
            assert!(holds(source, &data), "{source}");
        }
    }

    #[test]
    fn unset_fields_read_as_their_defaults() {
        let data = message(Fields::default());
        for source in [
            "!zeroed",
            "count == 0",
            "scale == 1.5",
            "state == 'IDLE'",
            "!has(count)",
            "!has(maybe)",
            "!has(left)",
            "!has(left.position)",
            "size(values) == 0",
        ] {
            assert!(holds(source, &data), "{source}");
        }
        // There's no default for an optional scalar, or a field of a table that isn't there.
        assert_eq!(
            at("maybe == 0", &data, None),
            Err(EvalError("maybe isn't set".into()))
        );
        assert_eq!(
            at("left.position == 0", &data, None),
            Err(EvalError("left isn't set".into()))
        );
    }

    #[test]
    fn has_and_size() {
        let data = sample();
        for source in [
            "has(left)",
            "has(left.position)",
            "has(values)",
            "has(values[2])",
            "!has(values[3])",
            "size(values) == 3",
            "size(label) == 5",
            "size(segment.ids) == 3",
            "has(pose)",
        ] {
            assert!(holds(source, &data), "{source}");
        }
        assert_eq!(
            at("values[3] > 0", &data, None),
            Err(EvalError("values[3] is past the end: there are 3".into()))
        );
        let data = message(Fields {
            maybe: Some(0),
            ..Default::default()
        });
        assert!(holds("has(maybe) && maybe == 0", &data));
    }

    #[test]
    fn arithmetic_and_precedence() {
        let data = sample();
        for source in [
            "1 + 2 * 3 == 7",
            "(1 + 2) * 3 == 9",
            "7 / 2 == 3.5",
            "trunc(7 / 2) == 3",
            "7 % 4 == 3",
            "7.5 % 2 == 1.5",
            "-count == -7",
            "!(count == 1)",
            "count > 1 || 1 / 0 > 0",
            "true || false && false",
            "count == 7 ? heading > 2 : heading < 2",
            "(false ? 1 : true ? 2 : 3) == 2",
            "(false ? 1 : 2.5) == 2.5",
            "big - 1 == 18446744073709551614",
            "values[0] + values[1] == values[2]",
        ] {
            assert!(holds(source, &data), "{source}");
        }
        assert_eq!(
            at("count % 0 == 1", &data, None),
            Err(EvalError("% by zero".into()))
        );
    }

    #[test]
    fn functions() {
        let data = sample();
        for source in [
            "abs(-3) == 3",
            "abs(left.velocity) == 2.5",
            "near(sin(pi / 2), 1, 1e-12)",
            "near(cos(0), 1, 1e-12)",
            "near(atan2(1, 1), pi / 4, 1e-12)",
            "near(hypot(3, 4), 5, 1e-12)",
            "near(sqrt(16), 4, 0)",
            "pow(2, 10) == 1024",
            "near(exp(log(5)), 5, 1e-12)",
            "near(log10(1000), 3, 1e-12)",
            "near(degrees(pi), 180, 1e-9)",
            "near(radians(90), pi / 2, 1e-12)",
            "near(wrap(3 * pi), pi, 1e-9)",
            "near(wrap(-pi), pi, 1e-9)",
            "near(wrap(radians(350)), radians(-10), 1e-9)",
            "floor(-1.5) == -2 && ceil(-1.5) == -1 && round(2.5) == 3 && trunc(-1.5) == -1",
            "sign(-4) == -1 && sign(0) == 0",
            "min(3, 1, 2) == 1 && max(3, 1, 2.5) == 3",
            "clamp(5, 0, 2) == 2 && clamp(-1, 0, 2) == 0",
            "isnan(nan) && isinf(inf) && isfinite(heading)",
            "nan != nan && !(nan == nan) && !(nan < 1)",
            "near(asin(1), pi / 2, 1e-12) && acos(1) == 0 && atan(0) == 0 && tan(0) == 0",
        ] {
            assert!(holds(source, &data), "{source}");
        }
    }

    #[test]
    fn over_time() {
        let before = message(Fields {
            count: 3,
            heading: 1.0,
            ..Default::default()
        });
        let after = message(Fields {
            count: 7,
            heading: 2.0,
            ..Default::default()
        });
        assert_eq!(at("changed(count)", &after, Some(&before)), Ok(true));
        assert_eq!(at("changed(count)", &after, Some(&after)), Ok(false));
        assert_eq!(at("changed(count)", &after, None), Ok(false));
        assert_eq!(at("delta(count) == 4", &after, Some(&before)), Ok(true));
        // Half a second apart.
        assert_eq!(at("rate(heading) == 2", &after, Some(&before)), Ok(true));
        assert_eq!(at("isnan(delta(count))", &after, None), Ok(true));
        assert_eq!(at("near(age(), 0.25, 1e-9)", &after, None), Ok(true));
        assert_eq!(at("stale(200ms)", &after, None), Ok(true));
        assert_eq!(at("stale(1s)", &after, None), Ok(false));
        assert!(compile("stale(1s)").unwrap().timed());
        assert!(!compile("count > 1").unwrap().timed());
    }

    #[test]
    fn nothing_sent_yet() {
        let condition = compile("count > 1 || stale(1s)").unwrap();
        let input = Input {
            message: None,
            previous: None,
            now_ns: 5_000_000_000,
            last_ns: 0,
        };
        assert!(condition.eval(&input).is_err());
        let condition = compile("stale(1s)").unwrap();
        assert_eq!(condition.eval(&input), Ok(true));
    }

    #[test]
    fn reports_mistakes_where_they_are() {
        for (source, wanted) in [
            (
                "cuont > 1",
                "cuont > 1\n^^^^^ aos.robot_server.testing.Status has no field cuont. Did you mean count?",
            ),
            (
                "left.velcity > 1",
                "left.velcity > 1\n     ^^^^^^^ aos.robot_server.testing.Wheel has no field velcity. Did you mean velocity?",
            ),
            (
                "state == 'INTAKE'",
                "state == 'INTAKE'\n         ^^^^^^^^ aos.robot_server.testing.State has no value \"INTAKE\": it's one of IDLE, INTAKING, SHOOTING",
            ),
            (
                "sine(heading) > 0",
                "sine(heading) > 0\n^^^^ There's no function sine. Did you mean sin, size or asin?",
            ),
            ("count = 1", "count = 1\n      ^ Compare with =="),
            (
                "1 < count < 3",
                "1 < count < 3\n          ^ Comparisons don't chain: join them with &&",
            ),
            (
                "count",
                "count\n^^^^^ A condition is true or false, but this is an integer",
            ),
            (
                "zeroed > false",
                "zeroed > false\n^^^^^^^^^^^^^^ a bool can only be compared with == and !=",
            ),
            (
                "count ? 1 : 2",
                "count ? 1 : 2\n^^^^^ This is an integer, where a bool goes",
            ),
            (
                "zeroed ? 1 : 'a' == 'a'",
                "zeroed ? 1 : 'a' == 'a'\n         ^^^^^^^^^^^^^^ ?:'s choices must be the same type, not an integer and a bool",
            ),
            (
                "sin(zeroed) > 0",
                "sin(zeroed) > 0\n    ^^^^^^ This is a bool, where a number goes",
            ),
            (
                "pow(2) > 1",
                "pow(2) > 1\n^^^^^^ pow() takes 2 arguments, not 1",
            ),
            (
                "left > 1",
                "left > 1\n^^^^ This is a table or struct, not a value: pick a field of it, or use has() or size()",
            ),
            (
                "values.x > 1",
                "values.x > 1\n       ^ This is a vector: index it, like [0], before .x",
            ),
            (
                "count.x > 1",
                "count.x > 1\n      ^ This has no fields, so no .x",
            ),
            (
                "stale(500)",
                "stale(500)\n      ^^^ stale() takes a duration, like stale(500ms)",
            ),
            (
                "stale(5min)",
                "stale(5min)\n       ^^^ min isn't a unit: durations are in ns, us, ms or s",
            ),
            (
                "count > 500ms",
                "count > 500ms\n        ^^^^^ Durations only go in stale(), like stale(500ms)",
            ),
            (
                "has(1)",
                "has(1)\n    ^ has() takes a field, like has(left.position)",
            ),
            ("", "\n^ The condition is empty"),
            (
                "label == 'open",
                "label == 'open\n         ^^^^^ This string isn't closed",
            ),
            (
                "count > 1 count",
                "count > 1 count\n          ^^^^^ Expected an operator, or the end of the condition",
            ),
        ] {
            assert_eq!(compile(source).unwrap_err(), wanted, "{source}");
        }
    }

    #[test]
    fn rejects_corrupt_messages() {
        let condition = compile("count > 1").unwrap();
        for data in [
            vec![],
            vec![200, 0, 0, 0],
            vec![4, 0, 0, 0, 255, 255, 255, 127],
        ] {
            let result = condition.eval(&Input {
                message: Some(Message {
                    data: &data,
                    monotonic_ns: 0,
                }),
                previous: None,
                now_ns: 0,
                last_ns: 0,
            });
            assert!(result.is_err(), "{data:?}");
        }
    }

    #[test]
    fn summarizes_the_fields_read() {
        let condition = compile("abs(heading) > 5 && count > delta(count) && has(left)").unwrap();
        assert_eq!(condition.fields(), vec!["heading", "count"]);
        let mut summary = Summary::new(&condition);
        for (count, heading) in [(1, -2.0), (4, 3.5), (2, 0.0)] {
            summary.add(&message(Fields {
                count,
                heading,
                ..Default::default()
            }));
        }
        assert_eq!(
            summary.fields(),
            &[
                FieldSummary {
                    field: "heading".into(),
                    last: Some("0".into()),
                    min: Some(-2.0),
                    max: Some(3.5),
                },
                FieldSummary {
                    field: "count".into(),
                    last: Some("2".into()),
                    min: Some(1.0),
                    max: Some(4.0),
                },
            ]
        );
    }
}
