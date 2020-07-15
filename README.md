# MPF-SPIKE Bridge
Bridge between the Mission Pinball Framework and Stern SPIKE pinball machines.

This is the code that runs on the SD card installed in the SPIKE CPU node which
allows a remote system (such as MPF) to control the pinball machine.

Details & Instructions:
http://docs.missionpinball.org/en/latest/hardware/spike

## Cross Compiling for Spike 1 (if you made changes)

```
cargo install cross
cross build --target armv5te-unknown-linux-musleabi --release
cp target/armv5te-unknown-linux-musleabi/release/mpf-spike bridge-spike1
```

## Cross Compiling for Spike 2 (if you made changes)

```
cargo install cross
cross build --target armv7-unknown-linux-musleabihf --release
cp target/armv7-unknown-linux-musleabihf/release/mpf-spike bridge-spike2
```

## Installing
Copy the bridge to your SPIKE SD card at ``/bin/bridge``.
The compiled binaries are located in this directory and are called ``bridge_spike1`` and ``bridge_spike2``.
See http://docs.missionpinball.org/en/latest/hardware/spike for details.
