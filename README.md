    ███╗   ███╗███████╗██████╗ ████████╗ ██████╗  ██████╗ ██╗     
    ████╗ ████║██╔════╝██╔══██╗╚══██╔══╝██╔═══██╗██╔═══██╗██║     
    ██╔████╔██║█████╗  ██║  ██║   ██║   ██║   ██║██║   ██║██║     
    ██║╚██╔╝██║██╔══╝  ██║  ██║   ██║   ██║   ██║██║   ██║██║     
    ██║ ╚═╝ ██║███████╗██████╔╝   ██║   ╚██████╔╝╚██████╔╝███████╗
    ╚═╝     ╚═╝╚══════╝╚═════╝    ╚═╝    ╚═════╝  ╚═════╝ ╚══════╝

Linux tool for the Mega Everdrive USB port thingy

Talks to a Mega Everdrive Core or Pro over its USB serial port.

# Building

```
make
```

# Usage

```
medtool -p <port> -m <mode> [-e]
```

`-p` is the serial port, usually `/dev/ttyACM0`.

`-m` picks what to do:

- `terminal` — bridge the everdrive fifo to a Unix socket at `/tmp/medtool`.
  Connect with `minicom -D unix#/tmp/medtool`. Characters printed from the
  MegaDrive side will show up in minicom and you can see characters into the
  fifo by typing. Code on the megadrive side can then collect it.
- `vdc` — read the voltage rails (5V, 2.5V, 1.2V, battery). wip.
- `rtc` — read the real-time clock. wip.

`-e` echoes incoming serial data to stdout when in terminal mode.

# Links

- The original/official tool code -> https://github.com/krikzz/mega-ed-pub/
