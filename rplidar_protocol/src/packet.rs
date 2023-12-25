const START_FLAG: u8 = 0xA5;

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Command {
    Stop = 0x25,
    Reset = 0x40,
    Scan = 0x20,
    ExpressScan = 0x82,
    ForceScan = 0x21,
    GetInfo = 0x50,
    GetHealth = 0x52,
    GetSampleRate = 0x59,
    GetLidarConf = 0x84,
}

impl Command {
    fn parse(command: u8) -> Option<Self> {
        if command == Command::Stop as u8 {
            Some(Command::Stop)
        } else if command == Command::Reset as u8 {
            Some(Command::Reset)
        } else if command == Command::Scan as u8 {
            Some(Command::Scan)
        } else if command == Command::ExpressScan as u8 {
            Some(Command::ExpressScan)
        } else if command == Command::ForceScan as u8 {
            Some(Command::ForceScan)
        } else if command == Command::GetInfo as u8 {
            Some(Command::GetInfo)
        } else if command == Command::GetHealth as u8 {
            Some(Command::GetHealth)
        } else if command == Command::GetSampleRate as u8 {
            Some(Command::GetSampleRate)
        } else if command == Command::GetLidarConf as u8 {
            Some(Command::GetLidarConf)
        } else {
            None
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PacketHeader {
    start_flag: u8,
    command: Command,
}

impl PacketHeader {
    fn new(start_flag: u8, command: Command) -> Self {
        Self {
            start_flag,
            command,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PacketBody {
    payload_size: u8,
    payload: Vec<u8>,
    checksum: u8,
}

impl PacketBody {
    fn new(payload_size: u8, payload: Vec<u8>, checksum: u8) -> Self {
        Self {
            payload_size,
            payload,
            checksum,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Request {
    OnlyHeader(PacketHeader),
    Full(PacketHeader, PacketBody),
}

impl Request {
    pub fn only_command(command: Command) -> Self {
        Self::from_header(PacketHeader::new(START_FLAG, command))
    }
    pub fn with_payload(command: Command, payload: Vec<u8>) -> Self {
        let payload_size = payload.len() as u8;
        let checksum = calculate_checksum(START_FLAG, command, payload_size, &payload);
        Self::from_header_body(
            PacketHeader::new(START_FLAG, command),
            PacketBody::new(payload_size, payload, checksum),
        )
    }
    fn from_header(header: PacketHeader) -> Self {
        Self::OnlyHeader(header)
    }
    fn from_header_body(header: PacketHeader, body: PacketBody) -> Self {
        Self::Full(header, body)
    }

    pub fn parse(bytes: &[u8]) -> Result<(Self, &[u8]), PacketError> {
        let mut iter = bytes.iter();
        let mut packet_size = 0usize;

        // Parse `start_flag`
        let start_flag = iter.next().ok_or(PacketError::MissingStartFlag)?;
        if *start_flag != START_FLAG {
            return Err(PacketError::MissingStartFlag);
        }
        packet_size += 1;

        // Parse `command` field
        let command = *iter.next().ok_or(PacketError::MissingCommand)?;
        let command = Command::parse(command).ok_or(PacketError::UnknownCommand { command })?;
        packet_size += 1;

        parse_body(&command, iter.as_slice()).map(|(packet, body_size)| {
            packet_size += body_size;
            (packet, &bytes[packet_size..])
        })
    }
}

#[derive(Debug, PartialEq)]
pub enum PacketError {
    MissingStartFlag,
    MissingCommand,
    MissingPayloadSize,
    MissingChecksum,
    IncompletePayload,
    UnknownCommand { command: u8 },
    InvalidChecksum { provided: u8, calculated: u8 },
}

fn parse_body(command: &Command, bytes: &[u8]) -> Result<(Request, usize), PacketError> {
    let mut body_size = 0usize;
    let request = match *command {
        Command::Stop
        | Command::Reset
        | Command::Scan
        | Command::ForceScan
        | Command::GetInfo
        | Command::GetHealth
        | Command::GetSampleRate => {
            Request::from_header(PacketHeader::new(START_FLAG, command.to_owned()))
        }
        // Requests with payload
        Command::ExpressScan | Command::GetLidarConf => {
            let mut iter = bytes.into_iter();
            let payload_size = *iter.next().ok_or(PacketError::MissingPayloadSize)?;
            body_size += 1;
            let payload: Vec<u8> = iter
                .clone()
                .take(payload_size.into())
                .map(|byte| byte.to_owned())
                .collect::<Vec<u8>>();
            if payload.len() != payload_size.into() {
                return Err(PacketError::IncompletePayload);
            }
            body_size += Into::<usize>::into(payload_size);
            let checksum = *iter
                .skip(payload_size.into())
                .next()
                .ok_or(PacketError::MissingChecksum)?;
            check_checksum(
                START_FLAG,
                command.to_owned(),
                payload_size,
                &payload,
                checksum,
            )?;
            body_size += 1;
            Request::from_header_body(
                PacketHeader {
                    start_flag: START_FLAG,
                    command: command.to_owned(),
                },
                PacketBody {
                    payload_size,
                    payload,
                    checksum,
                },
            )
        }
    };
    return Ok((request, body_size));
}

fn check_checksum(
    start_flag: u8,
    command: Command,
    payload_size: u8,
    payload: &[u8],
    expected: u8,
) -> Result<(), PacketError> {
    let calculated = calculate_checksum(start_flag, command, payload_size, payload);
    if calculated != expected {
        return Err(PacketError::InvalidChecksum {
            provided: expected,
            calculated,
        });
    }
    Ok(())
}

fn calculate_checksum(start_flag: u8, command: Command, payload_size: u8, payload: &[u8]) -> u8 {
    let mut result = 0u8;
    result ^= start_flag;
    result ^= command as u8;
    result ^= payload_size;
    for byte in payload {
        result ^= byte;
    }
    return result;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calculate_valid_checksum() {
        let calculated = calculate_checksum(
            START_FLAG,
            Command::ExpressScan,
            4,
            &[0x48, 0x84, 0x60, 0x7f],
        );
        //    0b0000_0000 = 0x00
        //  ^ 0x1010_0101 = 0xA5 "START_FLAG"
        // => 0b1010_0101
        //  ^ 0b1000_0010 = 0x82 "COMMAND"
        // => 0b0010_0111
        //  ^ 0b0000_0100 = 0x04 "PAYLOAD_SIZE"
        // => 0b0010_0011
        //  ^ 0b0100_1000 = 0x48 "PAYLOAD[0]"
        // => 0b0110_1011
        //  ^ 0b1000_0100 = 0x84 "PAYLOAD[1]"
        // => 0b1110_1111
        //  ^ 0b0110_0000 = 0x60 "PAYLOAD[2]"
        // => 0b1000_1111
        //  ^ 0b0111_1111 = 0x7f "PAYLOAD[3]"
        // => 0b1111_0000
        let expected = 0b1111_0000u8;

        assert_eq!(calculated, expected);
    }

    #[test]
    fn check_valid_checksum() {
        let checksum = 0b1111_0000u8;
        let result = check_checksum(
            START_FLAG,
            Command::ExpressScan,
            4,
            &[0x48, 0x84, 0x60, 0x7f],
            checksum,
        );
        assert_eq!(result.unwrap(), ());
    }

    #[test]
    fn check_invalid_checksum() {
        let checksum = 0b1111_0001u8;
        let result = check_checksum(
            START_FLAG,
            Command::ExpressScan,
            4,
            &[0x48, 0x84, 0x60, 0x7f],
            checksum,
        );
        assert_eq!(
            result.unwrap_err(),
            PacketError::InvalidChecksum {
                provided: checksum,
                calculated: 0b1111_0000
            }
        );
    }

    #[test]
    fn build_header_only_request() {
        let expected = Request::OnlyHeader(PacketHeader::new(START_FLAG, Command::Stop));
        let build = Request::only_command(Command::Stop);
        assert_eq!(build, expected);
    }

    #[test]
    fn build_full_request() {
        let payload: Vec<u8> = vec![0x48, 0x84, 0x60, 0x7f];
        let expected = Request::Full(
            PacketHeader::new(START_FLAG, Command::ExpressScan),
            PacketBody::new(4, payload.clone(), 0b1111_0000),
        );
        let build = Request::with_payload(Command::ExpressScan, payload.clone());
        assert_eq!(build, expected);
    }

    #[test]
    fn parse_single_request() {
        let payload = vec![0x48, 0x84, 0x60, 0x7f];
        let mut bytes = vec![START_FLAG, Command::ExpressScan as u8, 0x04];
        bytes.extend_from_slice(&payload);
        bytes.push(0xf0);

        let expected = Request::Full(
            PacketHeader::new(START_FLAG, Command::ExpressScan),
            PacketBody::new(4, payload, 0b1111_0000),
        );
        if let Ok((parsed, rest)) = Request::parse(&bytes) {
            assert_eq!(parsed, expected);
            assert!(rest.len() == 0);
        } else {
            assert!(false);
        }
    }
}
