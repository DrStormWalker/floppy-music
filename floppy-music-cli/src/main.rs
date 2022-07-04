use std::error::Error;
use std::io::{stdin, stdout, Write};
use std::sync::Arc;
use std::thread::{self, sleep};
use std::time::Duration;

use midir::{Ignore, MidiInput, MidiInputPort, MidiOutput, MidiOutputPort};
use tokio::io::AsyncReadExt;
//use serialport::SerialPort;
use tokio::sync::{mpsc, Mutex};
use tokio_serial::{SerialPort, SerialStream};

#[tokio::main]
async fn main() {
    match run().await {
        Ok(_) => (),
        Err(err) => println!("Error: {}", err),
    }
}

fn get_midi_ports(
    midi_in: &MidiInput,
    midi_out: &MidiOutput,
) -> Result<(MidiInputPort, MidiOutputPort), Box<dyn Error>> {
    // Get an input port (read from console if multiple are available)
    let in_ports = midi_in.ports();
    let in_port = match in_ports.len() {
        0 => return Err("no input port found".into()),
        1 => {
            println!(
                "Choosing the only available input port: {}",
                midi_in.port_name(&in_ports[0]).unwrap()
            );
            in_ports[0].clone()
        }
        _ => {
            println!("\nAvailable input ports:");
            for (i, p) in in_ports.iter().enumerate() {
                println!("{}: {}", i, midi_in.port_name(p).unwrap());
            }
            print!("Please select input port: ");
            stdout().flush()?;
            let mut input = String::new();
            stdin().read_line(&mut input)?;
            in_ports
                .get(input.trim().parse::<usize>()?)
                .ok_or("invalid input port selected")?
                .clone()
        }
    };

    // Get an output port (read from console if multiple are available)
    let out_ports = midi_out.ports();
    let out_port = match out_ports.len() {
        0 => return Err("no output port found".into()),
        1 => {
            println!(
                "Choosing the only available output port: {}",
                midi_out.port_name(&out_ports[0]).unwrap()
            );
            out_ports[0].clone()
        }
        _ => {
            println!("\nAvailable output ports:");
            for (i, p) in out_ports.iter().enumerate() {
                println!("{}: {}", i, midi_out.port_name(p).unwrap());
            }
            print!("Please select output port: ");
            stdout().flush()?;
            let mut input = String::new();
            stdin().read_line(&mut input)?;
            out_ports
                .get(input.trim().parse::<usize>()?)
                .clone()
                .ok_or("invalid output port selected")?
                .clone()
        }
    };

    Ok((in_port, out_port))
}

fn get_serial_port() -> Result<SerialStream, Box<dyn Error>> {
    let ports = tokio_serial::available_ports()?;
    println!("{:?}", ports);

    let port = ports.first().unwrap();
    println!("{:?}", port);

    let port = SerialStream::open(&tokio_serial::new(port.port_name.clone(), 115_200))?;

    Ok(port)
}

async fn run() -> Result<(), Box<dyn Error>> {
    let mut midi_in = MidiInput::new("midir reading input")?;
    midi_in.ignore(Ignore::None);

    let midi_out = MidiOutput::new("My Test Output")?;

    let (in_port, out_port) = get_midi_ports(&midi_in, &midi_out)?;

    let (tx, mut rx) = {
        let (out_tx, mut out_rx) = mpsc::channel::<Vec<u8>>(100);
        let (in_tx, in_rx) = mpsc::channel(100);

        let mut serial_port = get_serial_port()?;

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

    println!("\nOpening output connection");
    let mut conn_out = midi_out.connect(&out_port, "midir-test")?;

    println!("\nOpening input connection");

    // _conn_in needs to be a named parameter, because it needs to be kept alive until the end of the scope
    let _conn_in = midi_in.connect(
        &in_port,
        "midir-read-input",
        move |stamp, message, _| {
            conn_out.send(message).unwrap();

            let _ = tx.blocking_send(message.to_vec());
            //println!("{}: {:?} (len = {})", stamp, message, message.len());
        },
        (),
    )?;

    loop {
        if let Some(msg) = rx.recv().await {
            println!("Received: {:?}", String::from_utf8(msg).unwrap());
        }
    }

    #[allow(unreachable_code)]
    Ok(())
}
