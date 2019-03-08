use std::thread;
use std::sync::mpsc;
//use std::io::{self};
use std::fs::File;
use std::thread::JoinHandle;
use std::time::Duration;
use timeout_readwrite::TimeoutReader;
use std::os::unix::io::AsRawFd;
use std::io::Error;
use std::sync::mpsc::Sender;
use termios::*;
use termios::os::linux::*;
use std::fs::OpenOptions;
use std::os::unix::fs::OpenOptionsExt;
use std::env;
use std::os::unix::io::FromRawFd;

extern crate termios;
extern crate nix;
#[cfg(not(test))]
extern crate libc;

enum SpiMessage {
    Poll {
    },
    DmdFrame {
        data: [u8; 2048]
    },
    BacklightLed {
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

#[cfg(not(test))]
mod ioctl {
    use nix::*;

    #[allow(dead_code)]
    const GPIO_STATE: u16 = 0x3C02;

    const GPIO_ON: u16 = 0x3C03;
    const GPIO_OFF: u16 = 0x3C04;

    const BACKLIGHT_MAGIC: u16 = 0x4001;

    ioctl_write_int_bad!(set_brightness, BACKLIGHT_MAGIC);

    #[allow(dead_code)]
    ioctl_read_bad!(get_gpio_state, GPIO_STATE, bool);

    ioctl_write_int_bad!(set_gpio_on, GPIO_ON);
    ioctl_write_int_bad!(set_gpio_off, GPIO_OFF);
}

#[cfg(test)]
mod ioctl {
    use nix::Error;

    pub unsafe fn set_brightness(_fd: i32, _value: i32) -> Result<(), Error> {
        Ok(())
    }

    #[allow(dead_code)]
    pub unsafe fn get_gpio_state(_fd: i32, _value: i32) -> Result<bool, Error> {
        Ok(false)
    }

    pub unsafe fn set_gpio_on(_fd: i32, _value: i32) -> Result<(), Error> {
        Ok(())
    }

    pub unsafe fn set_gpio_off(_fd: i32, _value: i32) -> Result<(), Error> {
        Ok(())
    }
}

#[cfg(test)]
mod libc {
    pub const O_NOCTTY: i32 = 1;
    pub const O_SYNC: i32 = 2;

    pub unsafe fn tcdrain(_fd: i32) {

    }

    pub unsafe fn tcflush(_fd: i32, _action: i32) {

    }

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
    use std::fs::File;
    use std::os::raw::c_int;
    use std::sync::mpsc::RecvTimeoutError;

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

    impl std::os::unix::io::AsRawFd for TestPipeReader {

        fn as_raw_fd(&self) -> c_int {
            return -1;
        }

    }

    impl std::os::unix::io::AsRawFd for TestPipeSender {

        fn as_raw_fd(&self) -> c_int {
            return -1;
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
            match pipe.recv_timeout(Duration::new(5, 0)) {
                Ok(value) => {data.push(value);},
                Err(RecvTimeoutError::Timeout) => {panic!("Got a timeout from thread")}
                Err(RecvTimeoutError::Disconnected) => {panic!("Thread disconnected")}
            }
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

        let backlight_fd = File::open("/dev/null").unwrap();

        let threads = run_threads(host_fd_in, host_fd_out, bus_fd_in, bus_fd_out, spi_fd_in, spi_fd_out, backlight_fd);

        // Test init
        write_to_pipe(&host_in_tx, vec![0x80, 0x02, 0xf1, 0x8d, 0x00]);
        let data = read_from_pipe(&bus_out_rx, 5);
        assert_eq!(data, [0x80, 0x02, 0xf1, 0x8d, 0x00]);

        // Send command to node (with response_len 12)
        write_to_pipe(&host_in_tx, vec![0x81, 0x02, 0xfe, 0x7f, 0x0c]);
        // Check that node got message
        let data = read_from_pipe(&bus_out_rx, 5);
        assert_eq!(data, [0x81, 0x02, 0xfe, 0x7f, 0x0c]);
        // Node sends response
        write_to_pipe(&bus_in_tx, vec![0x01, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        // Check that host got the response from the node
        let data = read_from_pipe(&host_out_rx, 12);
        assert_eq!(data, vec![0x01, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);

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

        // Poll (SPI and node dirty)
        write_to_pipe(&host_in_tx, vec![0x00]);
        let data = read_from_pipe(&bus_out_rx, 1);
        assert_eq!(data, [0x00]);
        // Node sends response
        write_to_pipe(&bus_in_tx, vec![0x08]);
        // SPI sends switches (one changed)
        write_to_pipe(&spi_in_tx, vec![0x03, 0, 0, 0, 0, 0, 0, 0]);
        // Check that host got the response from the node
        let data = read_from_pipe(&host_out_rx, 1);
        assert_eq!(data, [0xF0]);

        // Read switches from SPI
        write_to_pipe(&host_in_tx, vec![0x80, 0x02, 0x11, 0xFF, 0x0B]);
        // SPI sends switches
        write_to_pipe(&spi_in_tx, vec![0x03, 0, 0, 0, 0, 0, 0, 0]);
        // Check that host got the response
        let data = read_from_pipe(&host_out_rx, 10);
        assert_eq!(data, [0x03, 0, 0, 0, 0, 0, 0, 0, 0, 253]);

        // Poll (SPI no longer dirty but node dirty)
        write_to_pipe(&host_in_tx, vec![0x00]);
        let data = read_from_pipe(&bus_out_rx, 1);
        assert_eq!(data, [0x00]);
        // Node sends response
        write_to_pipe(&bus_in_tx, vec![0x08]);
        // SPI sends switches (one changed)
        write_to_pipe(&spi_in_tx, vec![0x03, 0, 0, 0, 0, 0, 0, 0]);
        // Check that host got the response from the node
        let data = read_from_pipe(&host_out_rx, 1);
        assert_eq!(data, [0x08]);

        // Set backlight
        write_to_pipe(&host_in_tx, vec![0x80, 0x04, 0x80, 0x00, 0xff, 0xff, 0x00]);
        // Could assert on ioctl here

        // Send DMD frame
        let mut dmd_frame = vec![0; 2048];
        dmd_frame[3] = 42;
        dmd_frame[4] = 23;
        let mut message = vec![0x80, 0x00, 0x90];
        message.append(&mut dmd_frame.clone());
        write_to_pipe(&host_in_tx, message);
        let data = read_from_pipe(&spi_out_rx, 2048);
        assert_eq!(data, dmd_frame);

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
    println!("MPF Spike Bridge!");

    // Parse args first
    let args: Vec<String> = env::args().collect();
    // First arg is serial speed
    let serial_speed;
    if args.len() >= 2 {
        match args[1].as_ref() {
            "230400" => {serial_speed = B230400},
            "460800" => {serial_speed = B460800},
            "576000" => {serial_speed = B576000},
            "921600" => {serial_speed = B921600},
            "1000000" => {serial_speed = B1000000},
            "1152000" => {serial_speed = B1152000},
            "1500000" => {serial_speed = B1500000},
            "2000000" => {serial_speed = B2000000},
            "2500000" => {serial_speed = B2500000},
            "3000000" => {serial_speed = B3000000},
            "3500000" => {serial_speed = B3500000},
            "4000000" => {serial_speed = B4000000},
            _ => {serial_speed = 0 as u32}
        }
    } else {
        serial_speed = 0 as u32;
    }

    let std_in;
    let std_out;
    unsafe {
        std_out = File::from_raw_fd(1);
        std_in = File::from_raw_fd(0);
    }

    // Switch baud rate
    let termios_old = Termios::from_fd(std_in.as_raw_fd()).unwrap();
    let mut termios = Termios::from_fd(std_in.as_raw_fd()).unwrap();
    termios.c_iflag &= !(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    termios.c_oflag &= !OPOST;
    termios.c_lflag &= !(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    termios.c_cflag &= !(CSIZE | PARENB);
    termios.c_cflag |= CS8;
    if serial_speed > 0 {
        cfsetospeed(&mut termios, serial_speed).unwrap();
        cfsetispeed(&mut termios, serial_speed).unwrap();
    }
    tcsetattr(std_in.as_raw_fd(), TCSAFLUSH, &termios).unwrap();

    let host_fd_in = TimeoutReader::new(std_in.try_clone().unwrap(), Duration::new(5, 0));
    let host_fd_out = std_out;

    // Open the bus
    let bus_fd = OpenOptions::new().write(true).read(true).custom_flags(libc::O_SYNC | libc::O_NOCTTY).open("/dev/ttyS4").unwrap();
    let mut termios = Termios::from_fd(bus_fd.as_raw_fd()).unwrap();
    cfsetospeed(&mut termios, B460800).unwrap();
    cfsetispeed(&mut termios, B460800).unwrap();

    termios.c_cflag |= CLOCAL | CREAD;    /* ignore modem controls */
    termios.c_cflag &= !CSIZE;
    termios.c_cflag |= CS8;         /* 8-bit characters */
    termios.c_cflag &= !PARENB;     /* no parity bit */
    termios.c_cflag &= !CSTOPB;     /* only need 1 stop bit */
    termios.c_cflag &= !CRTSCTS;    /* no hardware flow control */

    /* setup for non-canonical mode */
    termios.c_iflag &= !(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    termios.c_lflag &= !(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    termios.c_oflag &= !OPOST;

    /* fetch bytes as they become available */
    termios.c_cc[VMIN] = 1;
    termios.c_cc[VTIME] = 1;
    tcsetattr(bus_fd.as_raw_fd(), TCSANOW, &termios).unwrap();

    // TODO: set TIOCM_RTS | TIOCM_DTR on bus_fd
    let bus_fd2 = bus_fd.try_clone().unwrap();

    // Open SPI for local switches
    let spi_fd = OpenOptions::new().write(true).read(true).custom_flags(libc::O_SYNC | libc::O_NOCTTY).open("/dev/spi1").unwrap();

    // Open DMD
    let dmd_fd = OpenOptions::new().write(true).read(true).custom_flags(libc::O_SYNC | libc::O_NOCTTY).open("/dev/spi0").unwrap();

    // Open magic backlight device
    let backlight_fd = File::open("/dev/backlight").unwrap();

    // Open magic gpio device
    let gpio_fd = File::open("/dev/gpio").unwrap();

    unsafe {
        // From NODEBUS_Init
        ioctl::set_gpio_on(gpio_fd.as_raw_fd(), 0x8C).unwrap();
        ioctl::set_gpio_off(gpio_fd.as_raw_fd(), 0x8A).unwrap();

        // Enable nodebus power
        ioctl::set_gpio_on(gpio_fd.as_raw_fd(), 0x6B).unwrap();
        // Enable amp (disabled for now because we do not use it anyway)
        //ioctl::set_gpio_on(gpio_fd.as_raw_fd(), 0x6A).unwrap();

        // From NODEBUS_Init (only during open)
        //ioctl::set_gpio_on(gpio_fd.as_raw_fd(), 0x8E).unwrap();
        //ioctl::set_gpio_off(gpio_fd.as_raw_fd(), 0x8E).unwrap();
    }

    let threads = run_threads(host_fd_in, host_fd_out, bus_fd, bus_fd2, spi_fd, dmd_fd, backlight_fd);


    for thread in threads {
        thread.join().unwrap();
    }

    unsafe {
        // Disable nodebus power
        ioctl::set_gpio_off(gpio_fd.as_raw_fd(), 0x6B).unwrap();
        // Disable amp (disabled for now because we do not use it anyway)
        //ioctl::set_gpio_off(gpio_fd.as_raw_fd(), 0x6A).unwrap();
    }
    tcsetattr(std_in.as_raw_fd(), TCSAFLUSH, &termios_old).unwrap();
    println!("Resetting terminal mode and quitting.");
}


fn host_thread<HIn: std::io::Read + std::marker::Send>(host_fd_in: &mut HIn, bus_tx: &Sender<NodeBusMessage>, spi_tx: &Sender<SpiMessage>) -> Result<bool, Error> {
    // First read node byte
    let mut node = [0; 1];
    host_fd_in.read_exact(&mut node)?;
    match node[0] {
        0 => {
            // Poll node bus
            let message = NodeBusMessage::Poll {};
            bus_tx.send(message).unwrap();
            // Poll SPI
            let message = SpiMessage::Poll {};
            spi_tx.send(message).unwrap();
            Ok(false)
        },
        1 => {
            // Sleep in node bus
            let mut wait_ms = [0; 1];
            host_fd_in.read_exact(&mut wait_ms)?;
            let message = NodeBusMessage::Wait { wait_ms: wait_ms[0] };
            bus_tx.send(message).unwrap();
            Ok(false)
        },
        0xF5 => {
            // Exit
            Ok(true)
        },
        _ => {
            // Commands to nodes
            let mut len = [0; 1];
            host_fd_in.read_exact(&mut len)?;
            let mut cmd = [0; 1];
            host_fd_in.read_exact(&mut cmd)?;
            if node[0] == 0x80 && len[0] == 0 && cmd[0] == 0x90 {
                // DMD frame
                let mut data: [u8; 2048] = [0; 2048];
                host_fd_in.read_exact(&mut data)?;
                let message = SpiMessage::DmdFrame {
                    data
                };
                spi_tx.send(message).unwrap();
                Ok(false)
            } else {
                let data_len: usize = (len[0] - 2) as usize;
                let mut data: [u8; 256] = [0; 256];
                host_fd_in.read_exact(&mut data[0..(data_len as usize)])?;
                let mut checksum = [0; 1];
                host_fd_in.read_exact(&mut checksum)?;
                let mut response_len = [0; 1];
                host_fd_in.read_exact(&mut response_len)?;
                if node[0] == 0x80 && cmd[0] == 0x11 {
                    // Read local switches
                    let message = SpiMessage::ReadLocalSwitches {};
                    spi_tx.send(message).unwrap();
                    Ok(false)
                } else if node[0] == 0x80 && len[0] == 4 && cmd[0] == 0x80 {
                    // LEDs on local node/Backlight
                    let message = SpiMessage::BacklightLed {
                        brightness: data[0]
                    };
                    spi_tx.send(message).unwrap();
                    Ok(false)
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
                    Ok(false)
                }
            }
        }
    }
}


fn run_threads<HIn: std::io::Read + std::marker::Send + 'static, HOut: std::io::Write + std::marker::Send + 'static,
    BIn: std::io::Read + std::marker::Send + std::os::unix::io::AsRawFd + 'static, BOut: std::io::Write + std::marker::Send + std::os::unix::io::AsRawFd + 'static,
    SIn: std::io::Read + std::marker::Send + 'static, SOut: std::io::Write + std::marker::Send + 'static,
    Bl: std::marker::Send + std::os::unix::io::AsRawFd + 'static>
(host_fd_in: HIn, host_fd_out: HOut, bus_fd_in: BIn, bus_fd_out: BOut, spi_fd_in: SIn, dmd_fd: SOut, backlight_fd: Bl) -> Vec<JoinHandle<()>> {
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
            let received = bus_rx.recv();
            match received {
                Ok(message) => {
                    match message {
                        NodeBusMessage::BusMessage { node, cmd, len, data, checksum, response_len } => {
                            unsafe {
                                libc::tcflush(fd_in.as_raw_fd(), 0);
                            }
                            fd_out.write(&[node, len, cmd]).unwrap();
                            fd_out.write(&data[0..((len - 2) as usize)]).unwrap();
                            fd_out.write(&[checksum, response_len]).unwrap();
                            if response_len > 0 {
                                let mut response = [0; 256];
                                match fd_in.read_exact(&mut response[0..(response_len as usize)]) {
                                    Ok(_) => {
                                        // Forward response to host
                                        host_tx.send(Response::Message { data: response, len: response_len }).unwrap();
                                    },
                                    Err(_) => {
                                        // Send desync response with corrent length
                                        host_tx.send(Response::Message { data: [55; 256], len: response_len }).unwrap();
                                        // Consume whatever is one the bus (may block for up to 200ms)
                                        // TODO: implement tcsendbreak(fd, 0)
                                        let mut response = [0; 256];
                                        let _ = fd_in.read(&mut response);
                                    },
                                }
                            }
                        },
                        NodeBusMessage::Poll => {
                            fd_out.write(&[0x00]).unwrap();
                            let mut response = [0; 1];
                            match fd_in.read_exact(&mut response) {
                                Ok(_) => { host_tx.send(Response::Poll { result: response[0] }).unwrap(); },
                                Err(_) => { host_tx.send(Response::Poll { result: 55 }).unwrap(); },
                            }
                        },
                        NodeBusMessage::Wait { wait_ms } => {
                            thread::sleep(Duration::from_millis(wait_ms as u64));
                        },
                        NodeBusMessage::Exit => {
                            let _result = host_tx.send(Response::Exit {});
                            // result is intentionally ignored since we exit anyway
                            return;
                        },
                    }
                    unsafe {
                        libc::tcdrain(fd_out.as_raw_fd());
                    }
                }
                Err(_) => {
                    return;
                }
            }
        }
    });

    let host_stdin_handler = thread::spawn(move || {
        let mut host_fd_in = host_fd_in;
        loop {
            match host_thread(&mut host_fd_in, &bus_tx, &spi_tx) {
                Ok(done) => {if done {break;}},
                Err(..) => {break;},
            }
        }
        let _result = bus_tx.send(NodeBusMessage::Exit {});
        // Result is unused because we exit
        let _result = spi_tx.send(SpiMessage::Exit {});
        // Result is unused because we exit

    });

    let host_stdout_handler = thread::spawn(move || {
        let mut fd = host_fd_out;
        let mut poll_result_spi = PollResult::Unknown{};
        let mut poll_result_node = PollResult::Unknown{};
        loop {
            let received = host_rx.recv();
            match received {
                Ok(response) => {
                    match response {
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
                },
                Err(_) => {
                    return;
                }
            }
        }
    });

    let spi_handler = thread::spawn(move || {
        let mut last_switch_state: [u8; 8] = [0; 8];
        let mut fd_in = spi_fd_in;
        let mut fd_out = dmd_fd;
        loop {
            let response = spi_rx.recv();
            match response {
                Ok(message) => {
                    match message {
                        SpiMessage::Poll {} => {
                            // Read SPI here
                            let mut switch_state = [0; 8];
                            fd_in.read_exact(&mut switch_state).unwrap();
                            match host_tx2.send(Response::SpiPoll { dirty: switch_state != last_switch_state }) {
                                Ok(_) => {},
                                Err(_) => {return;}
                            }

                        },
                        SpiMessage::DmdFrame { data } => {
                            // Write to DMD via SPI
                            fd_out.write(&data).unwrap();
                        },
                        SpiMessage::BacklightLed { brightness } => {
                            // Control backlight LED
                            unsafe {
                                ioctl::set_brightness(backlight_fd.as_raw_fd(), brightness as i32 * 256).unwrap();
                            }
                        },
                        SpiMessage::ReadLocalSwitches => {
                            // Read local switches
                            let mut switch_state = [0; 8];
                            fd_in.read_exact(&mut switch_state).unwrap();
                            let mut checksum: u32 = 0;
                            last_switch_state = switch_state;
                            for switch in switch_state.iter() {
                                checksum += *switch as u32;
                            }
                            let mut data: [u8; 256] = [0; 256];
                            data[0..8].clone_from_slice(&switch_state);
                            data[8..10].clone_from_slice(&[0, 256_u16.wrapping_sub((checksum & 0xFF) as u16) as u8]);
                            match host_tx2.send(Response::Message { data, len: 10 }) {
                                Ok(_) => {},
                                Err(_) => {return;}
                            }
                        },
                        SpiMessage::Exit => {
                            return;
                        },
                    }
                },
                Err(_) => {
                    return;
                }
            }
        }
    });

    return vec![bus_handler, host_stdin_handler, host_stdout_handler, spi_handler];
}

