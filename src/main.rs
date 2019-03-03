use std::thread;
use std::sync::mpsc;
use std::io::{self};
use std::fs::File;
use std::thread::JoinHandle;
use std::time::Duration;
use timeout_readwrite::TimeoutReader;

enum SpiMessage {
    Poll {
    },
    DmdFrame {
        data: [u8; 1024]
    },
    LocalLed {
        brightness: u8
    },
    ReadLocalSwitches,
    Exit
}

enum PollResult {
    Unknown,
    Dirty {
        result: u8
    },
    Clean
}

enum NodeBusMessage {
    BusMessage {
        node: u8,
        len: u8,
        cmd: u8,
        data: [u8; 256],
        checksum: u8,
        response_len: u8
    },
    Poll,
    Wait {
        wait_ms: u8
    },
    Exit
}

enum Response {
    Message {
        data: [u8; 256],
        len: u8
    },
    Poll {
        result: u8
    },
    SpiPoll {
        dirty: bool
    },
    Exit
}

#[cfg(test)]
mod tests {
    use crate::run_threads;
    use std::sync::mpsc::Sender;
    use std::sync::mpsc::Receiver;
    use std::io::Write;
    use std::io::Read;
    use std::io::Error;
    use std::sync::mpsc;
    use std::time::Duration;

    struct TestPipeSender {
        sender: Sender<u8>
    }

    impl Write for TestPipeSender {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Error> {
            let len = buf.len();
            for i in (std::ops::Range { start: 0, end: len }) {
                self.sender.send(buf[i]).unwrap();
            }
            Ok(len)
        }

        fn flush(&mut self) -> Result<(), Error> {
            Ok(())
        }

    }

    struct TestPipeReader {
        receiver: Receiver<u8>
    }

    impl Read for TestPipeReader {

        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
            let len = buf.len();
            for i in (std::ops::Range { start: 0, end: len }) {
                buf[i] = self.receiver.recv().unwrap();
            }
            Ok(len)
        }
    }

    fn write_to_pipe(pipe: &Sender<u8>, data: Vec<u8>) {
        for v in data {
            pipe.send(v).unwrap();
        }
    }

    fn read_from_pipe(pipe: &Receiver<u8>, len: usize) -> Vec<u8> {
        let mut data = Vec::new();
        for _ in (std::ops::Range { start: 0, end: len }) {
            data.push(pipe.recv_timeout(Duration::new(5, 0)).unwrap());
        }
        return data;
    }

    #[test]
    fn it_works() {
        let (host_in_tx, host_in_rx) = mpsc::channel::<u8>();
        let host_fd_in = TestPipeReader{ receiver: host_in_rx };

        let (host_out_tx, host_out_rx) = mpsc::channel::<u8>();
        let host_fd_out = TestPipeSender{ sender: host_out_tx };

        let (bus_in_tx, bus_in_rx) = mpsc::channel::<u8>();
        let bus_fd_in = TestPipeReader{ receiver: bus_in_rx };

        let (bus_out_tx, bus_out_rx) = mpsc::channel::<u8>();
        let bus_fd_out = TestPipeSender{ sender: bus_out_tx };

        let (spi_in_tx, spi_in_rx) = mpsc::channel::<u8>();
        let spi_fd_in = TestPipeReader{ receiver: spi_in_rx };

        let (spi_out_tx, spi_out_rx) = mpsc::channel::<u8>();
        let spi_fd_out = TestPipeSender{ sender: spi_out_tx };

        let threads = run_threads(host_fd_in, host_fd_out, bus_fd_in, bus_fd_out, spi_fd_in, spi_fd_out);

        // Send command to node (with response_len 2)
        write_to_pipe(&host_in_tx, vec![0x81, 0x03, 0xf0, 0x10, 0x00, 0x02]);
        // Check that node got message
        let data = read_from_pipe(&bus_out_rx, 6);
        assert_eq!(data, [0x81, 0x03, 0xf0, 0x10, 0x00, 0x02]);
        // Node sends response
        write_to_pipe(&bus_in_tx, vec![0x01, 0x02]);
        // Check that host got the response from the node
        let data = read_from_pipe(&host_out_rx, 2);
        assert_eq!(data, [0x01, 0x02]);

        // Sleep
        write_to_pipe(&host_in_tx, vec![0x01, 0x10]);

        // Poll
        write_to_pipe(&host_in_tx, vec![0x00]);
        let data = read_from_pipe(&bus_out_rx, 1);
        assert_eq!(data, [0x00]);
        // Node sends response
        write_to_pipe(&bus_in_tx, vec![0x00]);
        // SPI sends switches
        write_to_pipe(&spi_in_tx, vec![0x00; 8]);
        // Check that host got the response from the node
        let data = read_from_pipe(&host_out_rx, 1);
        assert_eq!(data, [0x00]);

        // Poll (board 8 dirty)
        write_to_pipe(&host_in_tx, vec![0x00]);
        let data = read_from_pipe(&bus_out_rx, 1);
        assert_eq!(data, [0x00]);
        // Node sends response
        write_to_pipe(&bus_in_tx, vec![0x08]);
        // SPI sends switches
        write_to_pipe(&spi_in_tx, vec![0x00; 8]);
        // Check that host got the response from the node
        let data = read_from_pipe(&host_out_rx, 1);
        assert_eq!(data, [0x08]);

        // Poll (SPI dirty)
        write_to_pipe(&host_in_tx, vec![0x00]);
        let data = read_from_pipe(&bus_out_rx, 1);
        assert_eq!(data, [0x00]);
        // Node sends response
        write_to_pipe(&bus_in_tx, vec![0x00]);
        // SPI sends switches (one changed)
        write_to_pipe(&spi_in_tx, vec![0x02, 0, 0, 0, 0, 0, 0, 0]);
        // Check that host got the response from the node
        let data = read_from_pipe(&host_out_rx, 1);
        assert_eq!(data, [0xF0]);

        // Quit
        write_to_pipe(&host_in_tx, vec![0xF5]);

        for thread in threads {
           match thread.join() {
               Ok(_) => {},
               Err(_) => {panic!("Thread crashed")},
           }
        }
    }
}


fn main() {
    let host_fd_in = TimeoutReader::new(io::stdin(), Duration::new(5, 0));
    let host_fd_out = io::stdout();
    let bus_fd = File::open("/dev/null").unwrap();
    let bus_fd2 = bus_fd.try_clone().unwrap();
    let spi_fd = File::open("/dev/null").unwrap();
    let spi_fd2 = spi_fd.try_clone().unwrap();
    let threads = run_threads(host_fd_in, host_fd_out, bus_fd, bus_fd2, spi_fd, spi_fd2);

    for thread in threads {
        thread.join().unwrap();
    }
}

fn run_threads<HIn: std::io::Read + std::marker::Send + 'static, HOut: std::io::Write + std::marker::Send + 'static,
    BIn: std::io::Read + std::marker::Send + 'static, BOut: std::io::Write + std::marker::Send + 'static,
    SIn: std::io::Read + std::marker::Send + 'static, SOut: std::io::Write + std::marker::Send + 'static>
(host_fd_in: HIn, host_fd_out: HOut, bus_fd_in: BIn, bus_fd_out: BOut, spi_fd_in: SIn, spi_fd_out: SOut) -> Vec<JoinHandle<()>> {
    // Talk to the spike bus
    let (bus_tx, bus_rx) = mpsc::channel::<NodeBusMessage>();

    // Talk to the host/MPF
    let (host_tx, host_rx) = mpsc::channel::<Response>();
    let host_tx2 = host_tx.clone();

    // Talk to SPI/local stuff
    let (spi_tx, spi_rx) = mpsc::channel::<SpiMessage>();

    let bus_handler = thread::spawn(move || {
        let mut fd_in = bus_fd_in;
        let mut fd_out = bus_fd_out;
        loop {
            let received = bus_rx.recv().unwrap();
            match received {
                NodeBusMessage::BusMessage { node, cmd, len, data, checksum, response_len } => {
                    fd_out.write(&[node, len, cmd]).unwrap();
                    fd_out.write(&data[0..((len - 2) as usize)]).unwrap();
                    fd_out.write(&[checksum, response_len]).unwrap();
                    if response_len > 0 {
                        let mut response = [0; 256];
                        fd_in.read_exact(&mut response[0..(response_len as usize)]).unwrap();
                        host_tx.send(Response::Message{data: response, len: response_len}).unwrap();
                    }

                },
                NodeBusMessage::Poll => {
                    fd_out.write(&[0x00]).unwrap();
                    let mut response = [0; 1];
                    fd_in.read_exact(&mut response).unwrap();
                    host_tx.send(Response::Poll{result: response[0]}).unwrap();
                },
                NodeBusMessage::Wait { wait_ms } => {
                    thread::sleep(Duration::from_millis(wait_ms as u64));
                },
                NodeBusMessage::Exit => {
                    host_tx.send(Response::Exit{}).unwrap();
                    return;
                },
            }
        }
    });

    let host_stdin_handler = thread::spawn(move || {
        let mut fd = host_fd_in;
        loop {
            // First read node byte
            let mut node = [0; 1];
            fd.read_exact(&mut node).unwrap();
            match node[0] {
                0 => {
                    // Poll node bus
                    let message = NodeBusMessage::Poll{};
                    bus_tx.send(message).unwrap();
                    // Poll SPI
                    let message = SpiMessage::Poll {};
                    spi_tx.send(message).unwrap();
                },
                1 => {
                    // Sleep in node bus
                    let mut wait_ms = [0; 1];
                    fd.read_exact(&mut wait_ms).unwrap();
                    let message = NodeBusMessage::Wait{wait_ms: wait_ms[0]};
                    bus_tx.send(message).unwrap();
                },
                0xF5 => {
                    // Exit
                    bus_tx.send(NodeBusMessage::Exit{}).unwrap();
                    spi_tx.send(SpiMessage::Exit{}).unwrap();
                    return;
                },
                _ => {
                    // Commands to nodes
                    let mut len = [0; 1];
                    fd.read_exact(&mut len).unwrap();
                    let data_len: usize = (len[0] - 2) as usize;
                    let mut cmd = [0; 1];
                    fd.read_exact(&mut cmd).unwrap();
                    let mut data : [u8; 256] = [0; 256];
                    fd.read_exact(&mut data[0..(data_len as usize)]).unwrap();
                    let mut checksum = [0; 1];
                    fd.read_exact(&mut checksum).unwrap();
                    let mut response_len = [0; 1];
                    fd.read_exact(&mut response_len).unwrap();
                    if node[0] == 0x80 && cmd[0] == 0x11 {
                        // Read local switches
                        let message = SpiMessage::ReadLocalSwitches {};
                    } else if node[0] == 0x80 && len[0] == 4 && cmd[0] == 0x80 {
                        // LEDs on local node
                        let message = SpiMessage::LocalLed {
                            brightness: 0
                        };
                    } else if node[0] == 0x80 && len[0] == 0 && cmd[0] == 0x90 {
                        // DMD frame
                        let message = SpiMessage::DmdFrame {
                            data: [0; 1024]
                        };
                    } else {
                        // Forward to node bus
                        let message = NodeBusMessage::BusMessage {
                            node: node[0],
                            len: len[0],
                            cmd: cmd[0],
                            data,
                            checksum: checksum[0],
                            response_len: response_len[0]
                        };
                        bus_tx.send(message).unwrap();
                    }
                }
            }
        }
    });

    let host_stdout_handler = thread::spawn(move || {
        let mut fd = host_fd_out;
        let mut poll_result_spi = PollResult::Unknown{};
        let mut poll_result_node = PollResult::Unknown{};
        loop {
            let received = host_rx.recv().unwrap();
            match received {
                Response::SpiPoll { dirty } => {
                    if dirty {
                        poll_result_spi = PollResult::Dirty {
                            result: 0xF0
                        };
                    } else {
                        poll_result_spi = PollResult::Clean {};
                    }
                },
                Response::Message { data, len } => {
                    fd.write(&data[0..(len as usize)]).unwrap();
                },
                Response::Poll { result } => {
                    if result > 0 {
                        poll_result_node = PollResult::Dirty {
                            result
                        };
                    } else {
                        poll_result_node = PollResult::Clean {};
                    }
                },
                Response::Exit => { return },
            }
            match (&poll_result_spi, &poll_result_node) {
                (PollResult::Unknown, _) => {},
                (_, PollResult::Unknown) => {},
                (PollResult::Dirty { .. }, _) => {
                    fd.write(&[0xF0]).unwrap();
                    poll_result_spi = PollResult::Unknown {};
                    poll_result_node = PollResult::Unknown {};
                },
                (_, PollResult::Dirty { result }) => {
                    fd.write(&[*result]).unwrap();
                    poll_result_spi = PollResult::Unknown {};
                    poll_result_node = PollResult::Unknown {};
                },
                (_, PollResult::Clean) => {
                    fd.write(&[0]).unwrap();
                    poll_result_spi = PollResult::Unknown {};
                    poll_result_node = PollResult::Unknown {};
                },
            }
        }
    });

    let spi_handler = thread::spawn(move || {
        let mut last_switch_state: [u8; 8] = [0; 8];
        let mut fd_in = spi_fd_in;
        loop {
            let messsage = spi_rx.recv().unwrap();
            match messsage {
                SpiMessage::Poll {} => {
                    // Read SPI here
                    let mut switch_state = [0; 8];
                    fd_in.read_exact(&mut switch_state).unwrap();
                    host_tx2.send(Response::SpiPoll {dirty: switch_state != last_switch_state}).unwrap();
                },
                SpiMessage::DmdFrame { data } => {
                    // TODO: Write SPI here

                },
                SpiMessage::LocalLed { brightness } => {
                    // TODO: Write SPI here

                },
                SpiMessage::ReadLocalSwitches => {
                    // Read local switches
                    let mut switch_state = [0; 8];
                    let mut checksum: u32 = 0;
                    last_switch_state = switch_state;
                    for switch in switch_state.iter() {
                        checksum += *switch as u32;
                    }

                    fd_in.read_exact(&mut switch_state).unwrap();
                    let mut data:[u8; 256] = [0; 256];
                    data[0..8].clone_from_slice(&switch_state);
                    data[8..10].clone_from_slice(&[(checksum & 0xFF) as u8, 0]);
                    host_tx2.send(Response::Message { data: data, len: 10 }).unwrap();
                },
                SpiMessage::Exit => {
                    return;
                },
            }
        }
    });

//    bus_handler.join();
//    spi_handler.join();
//    host_stdin_handler.join();
//    host_stdout_handler.join();
    return vec![bus_handler, host_stdin_handler, host_stdout_handler, spi_handler];
}

