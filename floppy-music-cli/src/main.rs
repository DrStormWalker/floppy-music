mod cli;
mod web;

#[macro_use]
extern crate rocket;

use std::{
    fs,
    io::{self, Write},
    path::{Path, PathBuf},
    str::FromStr,
};

use anyhow::{anyhow, Result as AnyResult};
use clap::{clap_derive::ArgEnum, Args, Parser, Subcommand, ValueEnum};
use cli::CliArgs;
use floppy_music_middle::sequencer::MidiEngine;
use midir::{MidiOutput, MidiOutputPort};
use midly::{live::LiveEvent, Smf};
use tokio::{
    io::AsyncReadExt,
    join, select,
    sync::{
        mpsc::{self, channel},
        oneshot,
    },
};
use tokio_serial::SerialStream;
use web::WebArgs;

#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
#[clap(infer_subcommands(true))]
struct Cli {
    #[clap(short, long)]
    verbose: bool,

    #[clap(flatten)]
    common: CommonArgs,

    #[clap(subcommand)]
    command: Command,
}

#[derive(Debug, Args)]
pub struct CommonArgs {
    #[clap(short, long)]
    device_port: Option<String>,
}

#[derive(Debug, Subcommand)]
enum Command {
    Cli(CliArgs),
    Web(WebArgs),
}

#[derive(Copy, Clone, Debug)]
pub enum ProgramSignal {
    Cancelled,
}

#[tokio::main]
async fn main() -> AnyResult<()> {
    let args = Cli::parse();

    let (signal_tx, signal_rx) = oneshot::channel();
    let (completed_tx, completed_rx) = oneshot::channel();

    let program = tokio::spawn(async move {
        let result = match args.command {
            Command::Cli(cli_args) => cli::start(cli_args, args.common, signal_rx).await,
            Command::Web(web_args) => web::start(web_args, args.common, signal_rx).await,
        };

        completed_tx.send(());
        
        println!("Result: {:?}", result);

        result
    });

    let cancelled = select! {
        _ = completed_rx => false,
        _ = tokio::signal::ctrl_c() => true,
    };

    if cancelled {
        let _ = signal_tx.send(ProgramSignal::Cancelled);

        let result = program.await?;
        
        //println!("Result? {:?}", result);

        Err(anyhow!("Cancelled"))
    } else {
        Ok(())
    }
}
