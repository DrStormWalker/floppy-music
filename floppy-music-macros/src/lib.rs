use proc_macro::TokenStream;
use quote::quote;
use std::ops::Range;
use syn::{parse::Parse, parse_macro_input, Token};

struct MidiNoteInput {
    range: Range<usize>,
    a4_midi: usize,
    a4_freq: f32,
}
impl Parse for MidiNoteInput {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        Ok(MidiNoteInput {
            range: 0..128,
            a4_midi: 69,
            a4_freq: 440.0,
        })
    }
}

#[proc_macro]
pub fn midi_note_periods(input: TokenStream) -> TokenStream {
    let ast = parse_macro_input!(input as MidiNoteInput);

    let notes = ast
        .range
        .into_iter()
        .map(|note| ast.a4_freq as f32 * 2.0f32.powf((note as f32 - ast.a4_midi as f32) / 12.0))
        .map(|freq| 1.0 / freq * 1_000_000.0)
        .map(|period| period as u32)
        .collect::<Vec<_>>();

    let expanded = quote! {
        [
            #(#notes ,)*
        ]
    };

    TokenStream::from(expanded)
}
