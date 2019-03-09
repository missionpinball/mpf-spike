# MPF-SPIKE Bridge
Bridge between the Mission Pinball Framework and Stern SPIKE pinball machines.

This is the code that runs on the SD card installed in the SPIKE CPU node which
allows a remote system (such as MPF) to control the pinball machine.

Details & Instructions:
http://docs.missionpinball.org/en/latest/hardware/spike

## Cross Compiling (if you made changes)

```
# build image with rust version
sudo docker build --tag rust-armv5te-unknown-linux-musleabi .
# change into image
sudo docker run --rm -it -v "$(pwd)":/home/rust/src rust-armv5te-unknown-linux-musleabi
# compile (inside docker)
carge build --release
```

The finished binary will be in ``target/armv5te-unknown-linux-musleabi/release/mpf-spike``.

## Installing
Copy the bridge to your SPIKE SD card at ``/bin/bridge``.
A compiled binary is located in this directory called ``bridge``.
See http://docs.missionpinball.org/en/latest/hardware/spike for details.
