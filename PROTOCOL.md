# Imagimob Streaming Protocol Specification v1.0

2023-11-28

### 1. Packet format

This section describes how message and data payloads are packaged into packets depending on the transport protocol; serial, UDP or BLE.

#### 1.1. Serial transport

Over a serial connection, the payload of a packet is sent as is, followed by a carriage return and a line feed. Since all payloads except actual sensor data are sent as text strings, this allows direct REPL interaction with the device using a standard terminal.

The UART should be configured with a baud rate of 1000000 bits/s, 8 bits, no parity and 1 stop bit (8N1).

#### 1.2. UDP transport

To be written

#### 1.3. BLE transport

To be written

### 2. Payloads

This section describes the request payloads sent from the host (typically a PC) to the device (the microcontroller board) and the response payloads sent back from the device to the host.

#### 2.1. config?

The config? request is typically the first request sent by the host. When the device receives this request, it first cancels all subscriptions, i.e. stops all sensor data streaming, and then it sends a JSON string providing information and capabilities of the device.

##### Request

```
config?
```

##### Response

```
{
    "device_name": <device name>,
    "protocol_version": 1,
    "heartbeat_timeout": <heartbeat timeout>,
    "sensors": [
        {
            "channel": <channel>,
            "type": "<sensor type>",
            "datatype": "<data type>",
            "shape": <shape>,
            "rates": [ <rate>, <rate>, … ]
        }
    ]
}
```
- *device name*: User-friendly device name for easy identification.
- *heartbeat timeout*: The time in seconds after which the device stops transmitting data if no heartbeat is received.
- *channel*: Channel number 1-9.
- *sensor type*: User-friendly sensor type name in lowercase letters.
- *data type*: Any of `"u8"` `"s8"`, `"u16"`, `"s16"`, `"u32"`, `"s32"`, `"f32"`, `"f64"`. All multi byte types are sent little endian.
- *shape*: The shape of the sensor data in one packet as a list of dimensions, typically \[<*number of samples*>, <*number of features*>\].
- *rate*: A valid data rate in Hz.


##### Response example

```
{
    "device_name": "PSoC6",
    "protocol_version": 1,
    "heartbeat_timeout": 5,
    "sensors": [
        {
            "channel": 1,
            "type": "microphone",
            "datatype": "s16",
            "shape": [ 256, 2 ],
            "rates": [ 8000, 16000 ]
        },
        {
            "channel": 2,
            "type": "accelerometer",
            "datatype": "s16",
            "shape": [ 1, 3 ],
            "rates": [ 20, 50, 100, 200, 500, 1000 ]
        }
    ]
}
```

#### 2.2. subscribe

The host sends a subscribe request to start sensor data streaming. The arguments are:

- *channel*: The sensor channel to subscribe to. Given the config example above, this would be 1 to receive audio data and 2 to receive accelerometer data.
- *rate*: The requested data rate, which must be one of the rates given by the config response.

After receiving this request, the device either starts streaming sensor data or replies with an error message. Each sensor data packet starts with the character 'B' followed by the channel number (as an ASCII character, so channel 1 is given as the character '1'), followed by the binary data. The format and shape of the binary data, and thus implicitly also the length, was given by the config? response. For example, the total length of audio data in the config example above is 2 x 2 x 256 = 1024 bytes.

All multi-byte elements are sent little endian.

##### Request

```
subscribe,<channel>,<rate>
```

##### Request example

```
subscribe,1,16000
```

##### Response

```
B<channel><binary data>
B<channel><binary data>
…
```

or 

```
ERROR:<error message>
```

#### 2.3. unsubscribe

The host sends unsubscribe to stop sensor data streaming.

- *channel* (optional): The sensor channel to unsubscribe from. If omitted, all streaming is stopped.

##### Request

```
unsubscribe[,<channel>]
```

##### Response

```
OK
```

#### 2.4. heartbeat

The host must send heartbeat regularly to inform the device that it's still connected. If the device doesn\'t receive a heartbeat for the timeout time given by the config response, all ongoing sensor data streaming stops.

##### Request

```
heartbeat
```

or empty packet.

##### Response

None.
