# Wireshark dissector

`pieusb.lua` decodes the protocol these scanners speak: SCSI over an IEEE1284
link, behind a GL641USB parallel-port bridge. Nothing on the wire looks like
SCSI. Every CDB byte, every payload byte and every status byte is its own
one-byte vendor control transfer to a bridge register, and only reply data
travels over the bulk endpoint, so one INQUIRY is forty frames of
`URB_CONTROL` that Wireshark alone can tell you nothing about.

The dissector rebuilds the transaction stream from those register writes and
decodes both directions of every command the corpus and the library know.

    tshark -X lua_script:wireshark/pieusb.lua -r capture.pcapng -Y pieusb
    wireshark -X lua_script:wireshark/pieusb.lua capture.pcapng

To load it in every session, copy or symlink it into the personal plugin
directory (Help > About Wireshark > Folders names the one your build uses):

    mkdir -p ~/.local/lib/wireshark/plugins
    ln -s "$PWD/wireshark/pieusb.lua" ~/.local/lib/wireshark/plugins/

## What it decodes

Transport, per frame: the wakeup sequence and the IEEE1284 command that follows
it, the parallel control lines, each CDB and payload byte with its index, the
bulk size announce, and every status byte by name (`OK`, `READ`, `BUSY`,
`AGAIN`, `CHECK CONDITION`, ...).

Commands, once their payload is complete: MODE SELECT (resolution, passes,
depth, colour format, the quality bits and the autofocus flag), the WRITE
subcommands (SET SCAN FRAME in mm as well as 1/3600 inch, EXPOSURE,
HIGHLIGHT/SHADOW, CALIBRATION INFO, CyberView's refused CMD 17), SET GAIN
OFFSET, and SLIDE with each action's parameter read the way that action reads it
-- steps and their mm, a frame count, or a focus position.

Replies, once the bulk data has arrived: INQUIRY (including the model number by
name), READ STATE, GET PARAMETERS, REQUEST SENSE (with a plain-English meaning
for the sense triples this family returns), READ GAIN OFFSET, and the
calibration-info block. Image reads get their per-line channel tags, which is
how you see that lines arrive one per channel in rotation rather than in plane
blocks; the CCD mask read gets a running count of its zero bytes, which is the
image width in pixels.

## Filtering

Every frame of a transaction carries `pieusb.txn`, `pieusb.opcode` and
`pieusb.command`, so a filter selects a whole command rather than one byte of
one:

    pieusb.command == "MODE SELECT"     every frame of every MODE SELECT
    pieusb.txn == 42                    one transaction, end to end
    pieusb.summary                      only the frames that complete a decode
    pieusb.status == 0x02               CHECK CONDITION
    pieusb.sense.key != 0               a reply that reported an error
    pieusb.slide.action == 0x10         focus moves
    pieusb.mode.autofocus == 0x08       autofocus passes

`pieusb.summary` is the most useful of these on its own: it reduces a capture to
one line per decoded command and reply.

## Test capture

The corpus contains no INQUIRY reply -- both vendor applications ask the device
once, before capturing started -- so `make_inquiry_capture.py` synthesises a
usbmon capture of one INQUIRY transaction around the 184 bytes recorded in
`tests/reference/proscan4000_inquiry.txt`:

    python3 wireshark/make_inquiry_capture.py /tmp/inquiry.pcapng
    tshark -X lua_script:wireshark/pieusb.lua -r /tmp/inquiry.pcapng -Y pieusb

## Limits

- Written against Linux usbmon captures, which is what the corpus is. The fields
  it reads are the encapsulation-independent ones, so USBPcap captures should
  work, but none has been tried.
- Control transfers are matched to their completion by `usb.urb_id`. A capture
  that lost one half of a pair loses that status byte.
- A capture that starts mid-transaction resynchronises at the next wakeup
  sequence; the partial transaction before it is reported as far as it can be.
- Line tags need `bytes_per_line` from a GET PARAMETERS in the same capture, and
  are shown only while the tags actually land where that stride says. The
  shading-reference reads use a different stride and stop after their first tag.
- Replies are decoded from the bytes the capture kept. A short snaplen shows up
  as `pieusb.uncaptured`.
