use std::{
    error::Error,
    fs,
    io::{self, Write},
    path::{Path, PathBuf},
    str::FromStr,
};

use clap::{clap_derive::ArgEnum, Args, Parser, ValueEnum};
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

use crate::{CommonArgs, ProgramSignal};

#[derive(Debug, Args)]
pub struct CliArgs {
    #[clap(short, long)]
    filepath: PathBuf,

    #[clap(short, long, default_value = "0")]
    #[clap(use_value_delimiter(true))]
    tracks: Vec<usize>,

    #[clap(short, long = "all")]
    all_tracks: bool,
}

pub fn get_serial_port(port: Option<String>) -> Result<SerialStream, Box<dyn std::error::Error>> {
    let ports = tokio_serial::available_ports()?;

    let port = match port {
        Some(port) => port,
        None => match ports.len() {
            0 => return Err("No available serial ports".into()),
            1 => {
                println!(
                    "Choosing the only available serial port: {}",
                    ports[0].port_name
                );

                ports[0].port_name.clone()
            }
            _ => ports
                .get(
                    inquire::Select::new(
                        "Available serial ports:",
                        ports
                            .iter()
                            .map(|port| {
                                format!(
                                    "{} {}",
                                    port.port_name,
                                    match &port.port_type {
                                        serialport::SerialPortType::UsbPort(
                                            serialport::UsbPortInfo {
                                                serial_number: Some(serial_number),
                                                manufacturer: Some(manufacturer),
                                                product: Some(product),
                                                ..
                                            },
                                        ) => format!(
                                            "{} - {} {}",
                                            serial_number, manufacturer, product
                                        ),
                                        _ => format!("{:?}", port.port_type),
                                    }
                                )
                            })
                            .collect(),
                    )
                    .raw_prompt()
                    .unwrap()
                    .index,
                )
                .unwrap()
                .port_name
                .clone(),
        },
    };

    let port = SerialStream::open(&tokio_serial::new(port, 115_200))?;

    Ok(port)
}

fn get_midi_out() -> Result<(MidiOutput, MidiOutputPort), Box<dyn std::error::Error>> {
    let midi_out = MidiOutput::new("My Test Output")?;

    // Get an output port (read from console if multiple are available)
    let out_ports = midi_out.ports();
    let out_port = match out_ports.len() {
        0 => return Err("No available MIDI ports".into()),
        1 => {
            println!(
                "Choosing the only available output port: {}",
                midi_out.port_name(&out_ports[0]).unwrap()
            );

            out_ports[0].clone()
        }
        _ => out_ports
            .get(
                inquire::Select::new(
                    "Available output ports:",
                    out_ports
                        .iter()
                        .map(|out| midi_out.port_name(out).unwrap())
                        .collect(),
                )
                .raw_prompt()
                .unwrap()
                .index,
            )
            .unwrap()
            .clone(),
    };

    Ok((midi_out, out_port))
}

pub async fn start(
    args: CliArgs,
    common: CommonArgs,
    signal_rx: oneshot::Receiver<ProgramSignal>,
) -> Result<(), Box<dyn Error>> {
    let bytes = fs::read(args.filepath).unwrap();
    // let file = Smf::parse(&bytes).unwrap();

    //let (midi_out, out_port) = get_midi_out()?;

    let (pi_tx, mut pi_rx) = {
        let (out_tx, mut out_rx) = mpsc::channel::<Vec<u8>>(100);
        let (in_tx, in_rx) = mpsc::channel(100);

        let mut serial_port = get_serial_port(common.device_port)?;

        tokio::spawn(async move {
            loop {
                let mut buf = [0u8; 256];
                tokio::select! {
                    val = serial_port.read(&mut buf[..]) => {
                        if let Ok(len) = val {
                            let _ = in_tx.send(buf[..len].to_vec()).await;
                        }
                    }

                    Some(msg) = out_rx.recv() => {
                        let _ = serial_port.write(msg.as_slice());
                    }

                    else => {}
                }
            }
        });

        (out_tx, in_rx)
    };

    //let mut conn_out = midi_out.connect(out_port, "floppy-music").unwrap();

    let (tx, mut rx) = mpsc::channel::<Vec<u8>>(64);

    let passthrough_task = tokio::spawn(async move {
        while let Some(msg) = rx.recv().await {
            let event = LiveEvent::parse(&msg).unwrap();

            //let _ = conn_out.send(&msg);
            pi_tx.send(msg).await;
        }
    });

    tokio::spawn(async move {
        loop {
            if let Some(msg) = pi_rx.recv().await {
                print!("{}", String::from_utf8(msg).unwrap());
            }
        }
    });

    // let mut track_map = vec![None; file.tracks.len()];

    // if args.all_tracks {
    //     for i in 0..track_map.len() {
    //         track_map[i] = Some(tx.clone());
    //     }
    // } else {
    //     for i in args.tracks {
    //         if let Some(track) = track_map.get_mut(i) {
    //             *track = Some(tx.clone());
    //         }
    //     }
    // }

    let mut engine = MidiEngine::new();
    engine.play_to(&bytes, tx).await;

    let _signal = select! {
        _ = engine.until_stopped() => None,
        signal = signal_rx => Some(signal),
    };

    engine.stop();

    passthrough_task.await;

    Ok(())
}
