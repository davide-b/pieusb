--
-- Wireshark dissector for PIE / Reflecta (Pacific Image) film scanners.
--
-- These scanners are a SCSI scanner behind a GL641USB parallel-port bridge, so
-- nothing on the wire looks like SCSI: every CDB byte, every payload byte and
-- every status byte is a separate one-byte vendor control transfer to a bridge
-- register, and only the reply data travels over the bulk IN endpoint. One
-- INQUIRY is forty frames of URB_CONTROL.
--
-- This dissector rebuilds the transaction stream from those register writes and
-- decodes the commands and replies. The layout it decodes is documented in
-- src/pieusb/transport.py, src/pieusb/option.py, src/pieusb/inquiry.py and
-- captures/ProScan 4000/*.md.
--
-- Usage:
--   tshark -X lua_script:wireshark/pieusb.lua -r capture.pcapng
--   wireshark -X lua_script:wireshark/pieusb.lua capture.pcapng
-- or copy into ~/.local/lib/wireshark/plugins/ to load it always.
--
-- Every frame of a transaction carries pieusb.txn, pieusb.opcode and
-- pieusb.command, so "pieusb.command == \"MODE SELECT\"" or "pieusb.txn == 42"
-- selects a whole command rather than one byte of it.
--

local p_pieusb = Proto("pieusb", "PIE/Reflecta scanner (GL641USB bridge)")

-- ---------------------------------------------------------------- constants --

local REQ_REGISTER, REQ_BUFFER = 0x0C, 0x04

local P_SIZE, P_STATUS, P_CMD, P_CTRL, P_DATA = 0x0082, 0x0084, 0x0085, 0x0087, 0x0088

local PORTS = {
    [0x0082] = "bulk transfer size",
    [0x0084] = "SCSI status",
    [0x0085] = "SCSI command/data",
    [0x0087] = "parallel control",
    [0x0088] = "parallel data",
}

local WAKEUP = { 0xFF, 0xAA, 0x55, 0x00, 0xFF, 0x87, 0x78 }
local IEEE_SCSI, IEEE_RESET = 0xE0, 0x30
local IEEE_CMDS = { [0x30] = "RESET", [0xE0] = "SCSI", [0x40] = "RELOAD?" }

local OPCODES = {
    [0x00] = "TEST UNIT READY",
    [0x03] = "REQUEST SENSE",
    [0x08] = "READ",
    [0x0A] = "WRITE",
    [0x0F] = "GET PARAMETERS",
    [0x12] = "INQUIRY",
    [0x15] = "MODE SELECT",
    [0x17] = "CMD 17",
    [0x18] = "COPY",
    [0x1A] = "MODE SENSE",
    [0x1B] = "SCAN",
    [0xD1] = "SLIDE",
    [0xD2] = "CMD D2",
    [0xD7] = "READ GAIN OFFSET",
    [0xDC] = "SET GAIN OFFSET",
    [0xDD] = "READ STATE",
}

-- Little-endian 16-bit code in the first two bytes of a WRITE (0x0A) payload.
local SUBCMDS = {
    [0x12] = "SCAN FRAME",
    [0x13] = "EXPOSURE",
    [0x14] = "HIGHLIGHT/SHADOW",
    [0x15] = "SET SCAN HEAD",
    [0x93] = "SET EXP TIME",
    [0x94] = "SET OFFSET",
    [0x17] = "CMD 17 (CyberView)",
    [0x95] = "CALIBRATION INFO",
    [0x96] = "SET GAIN",
}

local STATUSES = {
    [0x00] = "OK",
    [0x01] = "READ",
    [0x02] = "CHECK CONDITION",
    [0x03] = "BUSY",
    [0x08] = "AGAIN",
    [0x88] = "FAIL",
    [0xFF] = "ERROR",
}

local SLIDE_ACTIONS = {
    [0x00] = "OFFSET FORWARD",
    [0x01] = "OFFSET BACK",
    [0x02] = "EJECT DOWN",
    [0x03] = "EJECT UP",
    [0x04] = "NEXT",
    [0x05] = "PREV",
    [0x10] = "INIT/FOCUS",
    [0x40] = "RELOAD",
}

local MODELS = {
    [0x002c] = "ProScan 4000",
    [0x002e] = "CrystalScan 3600",
    [0x002f] = "DigitDia 4000",
    [0x0030] = "CrystalScan 7200",
    [0x0036] = "ProScan 7200",
    [0x003a] = "DigitDia 6000 Multiple Slide Scanner",
    [0x0047] = "ProScan 10T",
    [0x0048] = "RPS 10M (Pacific Image PrimeFilm XAs)",
}

local FILTER_MASKS = {
    [0x02] = "red", [0x04] = "green", [0x08] = "blue", [0x01] = "neutral",
    [0x10] = "infrared",
}

local PASSES = {
    [0x04] = "green filter only",
    [0x80] = "one pass colour (RGB)",
    [0x90] = "one pass colour + infrared (RGBI)",
}

local COLOR_FORMATS = {
    [0x01] = "PIXEL (interleaved, untagged)",
    [0x02] = "LINE",
    [0x04] = "INDEX (two-byte channel tag per line)",
}

local SENSE_KEYS = {
    [0x00] = "NO SENSE", [0x01] = "RECOVERED ERROR", [0x02] = "NOT READY",
    [0x03] = "MEDIUM ERROR", [0x04] = "HARDWARE ERROR", [0x05] = "ILLEGAL REQUEST",
    [0x06] = "UNIT ATTENTION", [0x0B] = "ABORTED COMMAND",
}

-- Sense triples this family is known to return, key/asc/ascq -> meaning.
local SENSE_MEANING = {
    ["02/04/01"] = "lamp warming up",
    ["05/1a/00"] = "parameter list length error",
    ["05/26/80"] = "invalid field in parameter list (CMD 17 unsupported)",
    ["06/82/00"] = "calibration disable not granted (must calibrate)",
    ["0b/00/06"] = "command aborted (STOP SCAN)",
}

-- Channel tag prefixing every line in INDEX colour format.
local LINE_TAGS = {
    [0x5252] = "R", [0x4747] = "G", [0x4242] = "B", [0x4949] = "I",
}

-- Bytes of a bulk reply kept for structured decoding. The longest structured
-- reply in the protocol is the 184-byte INQUIRY; image and CCD-mask data is
-- measured but not retained.
local REPLY_CAP = 256
-- Line tags shown per bulk chunk. A 300 dpi chunk carries 72 of them and a
-- full-resolution one thousands; the first few are what tell you the rotation.
local MAX_TAGS = 24

local FRAME_UNITS_PER_INCH = 3600   -- SET SCAN FRAME coordinates
local OFFSET_STEPS_PER_INCH = 240   -- SLIDE film-offset steps

-- ------------------------------------------------------------------- fields --

local f = {}

f.txn        = ProtoField.uint32("pieusb.txn", "Transaction", base.DEC)
f.command    = ProtoField.string("pieusb.command", "Command")
f.opcode     = ProtoField.uint8("pieusb.opcode", "Opcode", base.HEX, OPCODES)
f.subcmd     = ProtoField.uint16("pieusb.subcmd", "Subcommand", base.HEX, SUBCMDS)
f.summary    = ProtoField.string("pieusb.summary", "Summary")

f.port       = ProtoField.uint16("pieusb.port", "Register", base.HEX, PORTS)
f.regval     = ProtoField.uint8("pieusb.value", "Value", base.HEX)
f.wakeup_idx = ProtoField.uint8("pieusb.wakeup", "Wakeup byte", base.DEC)
f.ieee       = ProtoField.uint8("pieusb.ieee1284", "IEEE1284 command", base.HEX, IEEE_CMDS)
f.nstrobe    = ProtoField.bool("pieusb.par_ctrl.nstrobe", "nSTROBE", 8, nil, 0x01)
f.ninit      = ProtoField.bool("pieusb.par_ctrl.ninit", "nINIT", 8, nil, 0x04)
f.status     = ProtoField.uint8("pieusb.status", "SCSI status", base.HEX, STATUSES)
f.announce   = ProtoField.uint32("pieusb.announce", "Announced reply size", base.DEC)

f.cdb        = ProtoField.bytes("pieusb.cdb", "CDB")
f.cdb_idx    = ProtoField.uint8("pieusb.cdb.index", "CDB byte", base.DEC)
f.cdb_len    = ProtoField.uint16("pieusb.cdb.length", "Length field", base.DEC)
f.payload    = ProtoField.bytes("pieusb.payload", "Payload")
f.payload_idx = ProtoField.uint8("pieusb.payload.index", "Payload byte", base.DEC)
f.reply      = ProtoField.bytes("pieusb.reply", "Reply")
f.reply_off  = ProtoField.uint32("pieusb.reply.offset", "Offset in reply", base.DEC)
f.reply_len  = ProtoField.uint32("pieusb.reply.length", "Bytes in this chunk", base.DEC)
f.reply_total = ProtoField.uint32("pieusb.reply.total", "Reply bytes so far", base.DEC)
f.uncaptured = ProtoField.uint32("pieusb.uncaptured", "Bytes not captured", base.DEC)

-- MODE SELECT
f.ms_resolution = ProtoField.uint16("pieusb.mode.resolution", "Resolution (dpi)", base.DEC)
f.ms_passes     = ProtoField.uint8("pieusb.mode.passes", "Passes", base.HEX, PASSES)
f.ms_depth      = ProtoField.uint8("pieusb.mode.color_depth", "Colour depth", base.HEX)
f.ms_format     = ProtoField.uint8("pieusb.mode.color_format", "Colour format", base.HEX, COLOR_FORMATS)
f.ms_byteorder  = ProtoField.uint8("pieusb.mode.byte_order", "Byte order", base.HEX)
f.ms_quality    = ProtoField.uint8("pieusb.mode.quality", "Quality", base.HEX)
f.ms_sharpen    = ProtoField.bool("pieusb.mode.sharpen", "sharpen", 8, nil, 0x02)
f.ms_skipshade  = ProtoField.bool("pieusb.mode.skip_shading", "skipShadingAnalysis", 8, nil, 0x08)
f.ms_fastir     = ProtoField.bool("pieusb.mode.fast_infrared", "fastInfrared", 8, nil, 0x80)
f.ms_autofocus  = ProtoField.uint8("pieusb.mode.autofocus", "Autofocus pass", base.HEX)
f.ms_halftone   = ProtoField.uint8("pieusb.mode.halftone", "Halftone pattern", base.DEC)
f.ms_threshold  = ProtoField.uint8("pieusb.mode.line_threshold", "Line threshold", base.DEC)

-- WRITE subcommands
f.w_length   = ProtoField.uint16("pieusb.write.length", "Payload length field", base.DEC)
f.w_filter   = ProtoField.uint16("pieusb.write.filter", "Filter", base.HEX)
f.w_value    = ProtoField.uint16("pieusb.write.value", "Value (%)", base.DEC)
f.sf_index   = ProtoField.uint16("pieusb.frame.index", "Frame index", base.DEC)
f.sf_x0      = ProtoField.uint16("pieusb.frame.x0", "x0", base.DEC)
f.sf_y0      = ProtoField.uint16("pieusb.frame.y0", "y0", base.DEC)
f.sf_x1      = ProtoField.uint16("pieusb.frame.x1", "x1", base.DEC)
f.sf_y1      = ProtoField.uint16("pieusb.frame.y1", "y1", base.DEC)
f.sf_size    = ProtoField.string("pieusb.frame.size", "Frame size")

-- SET GAIN OFFSET / READ GAIN OFFSET
f.go_exp_r   = ProtoField.uint16("pieusb.gain_offset.exp_time_r", "Exposure time R (timer counts)", base.DEC)
f.go_exp_g   = ProtoField.uint16("pieusb.gain_offset.exp_time_g", "Exposure time G (timer counts)", base.DEC)
f.go_exp_b   = ProtoField.uint16("pieusb.gain_offset.exp_time_b", "Exposure time B (timer counts)", base.DEC)
f.go_exp_i   = ProtoField.uint16("pieusb.gain_offset.exp_time_i", "Exposure time I (timer counts)", base.DEC)
f.go_off_r   = ProtoField.uint8("pieusb.gain_offset.offset_r", "Offset R", base.DEC)
f.go_off_g   = ProtoField.uint8("pieusb.gain_offset.offset_g", "Offset G", base.DEC)
f.go_off_b   = ProtoField.uint8("pieusb.gain_offset.offset_b", "Offset B", base.DEC)
f.go_off_i   = ProtoField.uint8("pieusb.gain_offset.offset_i", "Offset I", base.DEC)
f.go_gain_r  = ProtoField.uint8("pieusb.gain_offset.gain_r", "Gain R", base.DEC)
f.go_gain_g  = ProtoField.uint8("pieusb.gain_offset.gain_g", "Gain G", base.DEC)
f.go_gain_b  = ProtoField.uint8("pieusb.gain_offset.gain_b", "Gain B", base.DEC)
f.go_gain_i  = ProtoField.uint8("pieusb.gain_offset.gain_i", "Gain I", base.DEC)
f.go_light   = ProtoField.uint8("pieusb.gain_offset.light", "Light level", base.DEC)
f.go_extra   = ProtoField.uint8("pieusb.gain_offset.extra_entries", "extraEntries", base.DEC)
f.go_double  = ProtoField.uint8("pieusb.gain_offset.double_times", "doubleTimes", base.DEC)
f.go_sat_r   = ProtoField.uint16("pieusb.gain_offset.saturation_r", "Saturation level R", base.DEC)
f.go_sat_g   = ProtoField.uint16("pieusb.gain_offset.saturation_g", "Saturation level G", base.DEC)
f.go_sat_b   = ProtoField.uint16("pieusb.gain_offset.saturation_b", "Saturation level B", base.DEC)

-- SLIDE
f.sl_action  = ProtoField.uint8("pieusb.slide.action", "Action", base.HEX, SLIDE_ACTIONS)
f.sl_param   = ProtoField.uint8("pieusb.slide.parameter", "Parameter", base.DEC)
f.sl_byte3   = ProtoField.uint8("pieusb.slide.byte3", "Byte 3", base.HEX)
f.sl_meaning = ProtoField.string("pieusb.slide.meaning", "Meaning")

-- READ STATE
f.rs_frame     = ProtoField.uint8("pieusb.state.frame", "Frame index", base.DEC)
f.rs_focus     = ProtoField.uint8("pieusb.state.focus", "Focus position", base.DEC)
f.rs_focus_max = ProtoField.uint8("pieusb.state.focus_max", "Focus maximum", base.DEC)
f.rs_flags     = ProtoField.uint8("pieusb.state.flags", "Transport flags", base.HEX)
f.rs_medium    = ProtoField.bool("pieusb.state.medium_present", "medium present", 8, nil, 0x01)
f.rs_strip     = ProtoField.bool("pieusb.state.filmstrip", "filmstrip carrier", 8, nil, 0x04)
f.rs_byte7     = ProtoField.uint8("pieusb.state.byte7", "Byte 7 (bitfield, unexplained)", base.HEX)

-- GET PARAMETERS
f.gp_width     = ProtoField.uint16("pieusb.params.width", "Width (pixels)", base.DEC)
f.gp_lines     = ProtoField.uint16("pieusb.params.lines", "Lines", base.DEC)
f.gp_bpl       = ProtoField.uint16("pieusb.params.bytes_per_line", "Bytes per line (per plane, tag excluded)", base.DEC)
f.gp_avail     = ProtoField.uint16("pieusb.params.available_lines", "Lines available in device buffer", base.DEC)

-- REQUEST SENSE
f.sn_key       = ProtoField.uint8("pieusb.sense.key", "Sense key", base.HEX, SENSE_KEYS)
f.sn_asc       = ProtoField.uint8("pieusb.sense.asc", "Additional sense code", base.HEX)
f.sn_ascq      = ProtoField.uint8("pieusb.sense.ascq", "Additional sense code qualifier", base.HEX)
f.sn_meaning   = ProtoField.string("pieusb.sense.meaning", "Meaning")

-- INQUIRY
f.iq_vendor    = ProtoField.string("pieusb.inquiry.vendor", "Vendor")
f.iq_product   = ProtoField.string("pieusb.inquiry.product", "Product")
f.iq_revision  = ProtoField.string("pieusb.inquiry.revision", "Revision")
f.iq_res_x     = ProtoField.uint16("pieusb.inquiry.max_resolution_x", "Maximum resolution X (dpi)", base.DEC)
f.iq_res_y     = ProtoField.uint16("pieusb.inquiry.max_resolution_y", "Maximum resolution Y (dpi)", base.DEC)
f.iq_scan_w    = ProtoField.uint16("pieusb.inquiry.max_scan_width", "Bed width (1/3600 in)", base.DEC)
f.iq_scan_h    = ProtoField.uint16("pieusb.inquiry.max_scan_height", "Bed height (1/3600 in)", base.DEC)
f.iq_filters   = ProtoField.uint8("pieusb.inquiry.filters", "Filters", base.HEX)
f.iq_depths    = ProtoField.uint8("pieusb.inquiry.color_depths", "Colour depths", base.HEX)
f.iq_formats   = ProtoField.uint8("pieusb.inquiry.color_formats", "Colour formats", base.HEX)
f.iq_caps      = ProtoField.uint8("pieusb.inquiry.capabilities", "Scan capabilities", base.HEX)
f.iq_optional  = ProtoField.uint8("pieusb.inquiry.optional_devices", "Optional devices", base.HEX)
f.iq_preview   = ProtoField.uint16("pieusb.inquiry.preview_resolution", "Preview resolution (dpi)", base.DEC)
f.iq_firmware  = ProtoField.string("pieusb.inquiry.firmware", "Firmware version")
f.iq_max_exp   = ProtoField.uint16("pieusb.inquiry.maximum_exposure", "Maximum exposure (advertised)", base.DEC)
f.iq_min_exp   = ProtoField.uint16("pieusb.inquiry.minimum_exposure", "Minimum exposure (advertised)", base.DEC)
f.iq_model     = ProtoField.uint16("pieusb.inquiry.model", "Model number", base.HEX, MODELS)
f.iq_production = ProtoField.string("pieusb.inquiry.production", "Production code")
f.iq_timestamp = ProtoField.string("pieusb.inquiry.timestamp", "Timestamp")
f.iq_signature = ProtoField.string("pieusb.inquiry.signature", "Signature")

-- calibration info
f.ci_entries   = ProtoField.uint8("pieusb.calibration.entries", "Entries", base.DEC)
f.ci_entrysize = ProtoField.uint8("pieusb.calibration.entry_size", "Entry size", base.DEC)
f.ci_type      = ProtoField.uint8("pieusb.calibration.type", "Entry type", base.HEX)
f.ci_lines     = ProtoField.uint8("pieusb.calibration.n_lines", "Lines", base.DEC)
f.ci_pixels    = ProtoField.uint16("pieusb.calibration.pixels_per_line", "Pixels per line", base.DEC)

-- image data
f.img_tag      = ProtoField.uint16("pieusb.line.tag", "Line tag", base.HEX, LINE_TAGS)
f.img_rotation = ProtoField.string("pieusb.line.rotation", "Channel rotation")

local all = {}
for _, v in pairs(f) do all[#all + 1] = v end
p_pieusb.fields = all

local e_check = ProtoExpert.new("pieusb.expert.check", "SCSI CHECK CONDITION",
                                expert.group.RESPONSE_CODE, expert.severity.WARN)
local e_sense = ProtoExpert.new("pieusb.expert.sense", "Sense data reports an error",
                                expert.group.RESPONSE_CODE, expert.severity.WARN)
local e_fail = ProtoExpert.new("pieusb.expert.fail", "Bridge reported a failure status",
                               expert.group.RESPONSE_CODE, expert.severity.ERROR)
local e_short = ProtoExpert.new("pieusb.expert.short", "Reply bytes were not captured",
                                expert.group.UNDECODED, expert.severity.NOTE)
local e_unknown = ProtoExpert.new("pieusb.expert.unknown_opcode", "Opcode not known to this dissector",
                                  expert.group.UNDECODED, expert.severity.NOTE)
local e_error = ProtoExpert.new("pieusb.expert.error", "Dissector error",
                                expert.group.MALFORMED, expert.severity.ERROR)
p_pieusb.experts = { e_check, e_sense, e_fail, e_short, e_unknown, e_error }

-- --------------------------------------------------------------- extractors --

local function extractor(name)
    local ok, fld = pcall(Field.new, name)
    if ok then return fld end
    return function() return nil end
end

local f_xfer    = extractor("usb.transfer_type")
local f_brq     = extractor("usb.setup.bRequest")
local f_bmrt    = extractor("usb.bmRequestType")
local f_wval    = extractor("usb.setup.wValue")
local f_frag    = extractor("usb.data_fragment")
local f_resp    = extractor("usb.control.Response")
local f_capdata = extractor("usb.capdata")
local f_urbid   = extractor("usb.urb_id")
local f_irpid   = extractor("usb.irp_id")
local f_ep      = extractor("usb.endpoint_address")
local f_urblen  = extractor("usb.urb_len")
local f_datalen = extractor("usb.data_len")
local f_bus     = extractor("usb.bus_id")
local f_devaddr = extractor("usb.device_address")

-- ------------------------------------------------------------------- helpers --

local function fival(fld)
    local fi = fld()
    if fi then return fi.value end
    return nil
end

local function firange(fld)
    local fi = fld()
    if not fi then return nil end
    local ok, rng = pcall(function() return fi.range end)
    if ok then return rng end
    return nil
end

-- Byte i (0-based) of a hex string built by the first pass.
local function hb(hex, i)
    local s = hex:sub(i * 2 + 1, i * 2 + 2)
    if #s < 2 then return nil end
    return tonumber(s, 16)
end

local function hle16(hex, i)
    local lo, hi = hb(hex, i), hb(hex, i + 1)
    if not lo or not hi then return nil end
    return lo + hi * 256
end

local function tohex(range, limit)
    local n = range:len()
    if limit and n > limit then n = limit end
    if n <= 0 then return "" end
    return tostring(range:range(0, n):bytes())
end

local function opname(op, sub)
    if not op then return "(no CDB)" end
    local name = OPCODES[op] or string.format("OPCODE 0x%02X", op)
    if op == 0x0A and sub then
        name = name .. ":" .. (SUBCMDS[sub] or string.format("SUB 0x%04X", sub))
    end
    return name
end

-- Names the set bits of `byte` in mask order. Bits the protocol has no name for
-- stay visible as a trailing hex remainder rather than being dropped.
local function bitlist(byte, names)
    local masks = {}
    for mask in pairs(names) do masks[#masks + 1] = mask end
    table.sort(masks)
    local out, left = {}, byte
    for _, mask in ipairs(masks) do
        if byte % (mask * 2) >= mask then
            out[#out + 1] = names[mask]
            left = left - mask
        end
    end
    if left ~= 0 then out[#out + 1] = string.format("+0x%02x", left) end
    if #out == 0 then return "none" end
    return table.concat(out, " ")
end

local DEPTH_BITS = { [0x01] = "1", [0x02] = "4", [0x04] = "8", [0x08] = "10",
                     [0x10] = "12", [0x20] = "16" }
local QUALITY_BITS = { [0x02] = "sharpen", [0x08] = "skipShadingAnalysis",
                       [0x80] = "fastInfrared" }
local CAPABILITY_BITS = { [0x08] = "DisableCal", [0x10] = "FastPreview",
                          [0x40] = "ExtendedCal", [0x80] = "PowerSave" }
local OPTIONAL_BITS = { [0x01] = "ADF", [0x02] = "TP", [0x04] = "TP1",
                        [0x80] = "MultiPageLoad" }
local FORMAT_BITS = { [0x01] = "Pixel", [0x02] = "Line", [0x04] = "Indexed" }

local function txt(range)
    local s = range:string()
    s = s:gsub("%z", " "):gsub("^%s+", ""):gsub("%s+$", "")
    return s
end

local function mm_frame(v)
    return v * 25.4 / FRAME_UNITS_PER_INCH
end

-- ------------------------------------------------------------- state (pass 1) --

local devs, recs, pending, txn_count

local function reset_state()
    devs, recs, pending, txn_count = {}, {}, {}, 0
end

reset_state()

function p_pieusb.init()
    -- Only the walking state is thrown away: `recs` is keyed by frame number and
    -- is what a re-dissection of an already-visited frame reads back.
    devs, pending, txn_count = {}, {}, 0
end

local function devstate()
    local bus, addr = fival(f_bus), fival(f_devaddr)
    local key = string.format("%s.%s", tostring(bus), tostring(addr))
    local st = devs[key]
    if not st then
        st = { run = {}, cur = nil, bpl = nil, indexed = nil, expect_calinfo = false }
        devs[key] = st
    end
    return st
end

local function urbkey()
    local id = fival(f_urbid) or fival(f_irpid)
    if id == nil then return nil end
    return tostring(id)
end

local function new_txn(st)
    txn_count = txn_count + 1
    st.cur = {
        n = txn_count, cmd = {}, op = nil, cdblen = nil, sub = nil,
        announced = 0, got = 0, reply = "", decoded = false, zeros = 0,
    }
    return st.cur
end

-- How far the tail of the parallel-data run matches the wakeup sequence.
local function wakeup_progress(run)
    local best = 0
    for k = 1, #WAKEUP do
        if #run < k then break end
        local ok = true
        for j = 1, k do
            if run[#run - k + j] ~= WAKEUP[j] then ok = false break end
        end
        if ok then best = k end
    end
    return best
end

local function analyze(pinfo)
    local xfer = fival(f_xfer)
    if xfer == nil then return nil end

    local brq, bmrt, wval = fival(f_brq), fival(f_bmrt), fival(f_wval)
    local frag, resp, cap = f_frag(), f_resp(), f_capdata()
    local st = devstate()
    local r

    -- Control OUT: one byte (or the 8-byte size announce) into a bridge register.
    if brq and frag then
        local port = wval
        local b = frag.range:range(0, 1):uint()

        if brq == REQ_REGISTER and port == P_DATA then
            local run = st.run
            -- CyberView X writes every data and control byte twice, so the
            -- wakeup sequence only matches once runs are collapsed.
            local fresh = (#run == 0) or (run[#run] ~= b)
            if fresh then
                run[#run + 1] = b
                if #run > 16 then table.remove(run, 1) end
            end
            local matched = false
            if fresh and #run > #WAKEUP then
                matched = true
                local base = #run - #WAKEUP - 1
                for k = 1, #WAKEUP do
                    if run[base + k] ~= WAKEUP[k] then matched = false break end
                end
            end
            if matched then
                if b == IEEE_SCSI then
                    local cur = new_txn(st)
                    r = { kind = "ieee", ieee = b, txn = cur.n }
                else
                    st.cur = nil
                    r = { kind = "ieee", ieee = b }
                end
            else
                local k = fresh and wakeup_progress(run) or 0
                if k > 0 then
                    r = { kind = "wakeup", idx = k }
                else
                    r = { kind = "data" }
                end
            end

        elseif brq == REQ_REGISTER and port == P_CTRL then
            r = { kind = "ctrl" }

        elseif brq == REQ_REGISTER and port == P_CMD then
            local cur = st.cur
            if not cur then return { kind = "orphan" } end
            local cmd = cur.cmd
            cmd[#cmd + 1] = b
            local n = #cmd
            if n <= 6 then
                r = { kind = "cdb", idx = n - 1, txn = cur.n }
                if n == 6 then
                    cur.op = cmd[1]
                    cur.cdblen = cmd[4] * 256 + cmd[5]
                    r.cdb = string.format("%02x%02x%02x%02x%02x%02x", cmd[1], cmd[2],
                                          cmd[3], cmd[4], cmd[5], cmd[6])
                    if cur.op == 0x08 and st.expect_calinfo then
                        cur.calinfo = true
                        st.expect_calinfo = false
                    end
                end
            else
                r = { kind = "payload", idx = n - 7, txn = cur.n }
                if n == 8 and cur.op == 0x0A then
                    cur.sub = cmd[7] + cmd[8] * 256
                end
                if cur.cdblen and (n - 6) == cur.cdblen then
                    local parts = {}
                    for i = 7, n do parts[#parts + 1] = string.format("%02x", cmd[i]) end
                    local hex = table.concat(parts)
                    r.payload = hex
                    -- Scan context the reply decoders need later.
                    if cur.op == 0x15 and #hex >= 14 then
                        st.indexed = (hb(hex, 6) == 0x04)
                    elseif cur.op == 0x0A and cur.sub == 0x95 then
                        st.expect_calinfo = true
                    end
                end
            end
            r.op, r.sub = cur.op, cur.sub

        elseif brq == REQ_BUFFER and port == P_SIZE then
            local cur = st.cur
            local size
            if frag.range:len() >= 8 then
                size = frag.range:range(4, 4):le_uint()
            end
            r = { kind = "announce", size = size }
            if cur then
                r.txn, r.op, r.sub = cur.n, cur.op, cur.sub
                -- A single READ can need several announce+bulk rounds.
                cur.announced = cur.announced + (size or 0)
            end
        else
            r = { kind = "other" }
        end
        if r then r.port = port end
        return r
    end

    -- Control IN: remember which register a submitted read is for.
    if brq and bmrt and bmrt >= 0x80 then
        local key = urbkey()
        if key then pending[key] = wval end
        return nil
    end

    -- Control IN completion: the register's value.
    if resp then
        local key = urbkey()
        local port = key and pending[key] or nil
        if key then pending[key] = nil end
        if port ~= P_STATUS then return nil end
        local cur = st.cur
        r = { kind = "status", port = port }
        if cur then r.txn, r.op, r.sub = cur.n, cur.op, cur.sub end
        return r
    end

    -- Bulk IN: reply data.
    if cap and xfer == 3 then
        local ep = fival(f_ep) or 0
        if ep < 0x80 then return nil end
        local cur = st.cur
        local got = cap.range:len()
        local urblen, datalen = fival(f_urblen), fival(f_datalen)
        r = { kind = "bulk", len = got }
        if urblen and datalen and urblen > datalen then
            r.missing = urblen - datalen
        end
        if not cur then return r end
        r.txn, r.op, r.sub = cur.n, cur.op, cur.sub
        r.off = cur.got
        cur.got = cur.got + got + (r.missing or 0)
        r.total = cur.got

        if #cur.reply < REPLY_CAP * 2 then
            cur.reply = cur.reply .. tohex(cap.range, REPLY_CAP)
        end
        if cur.op == 0x18 then
            -- CCD mask: a zero marks a pixel the shading reference covers.
            local raw = cap.range:raw()
            local _, zeros = raw:gsub("%z", "")
            cur.zeros = cur.zeros + zeros
            r.zeros = cur.zeros
        end
        if not cur.decoded and cur.announced > 0 and cur.announced <= REPLY_CAP
                and cur.got >= cur.announced then
            cur.decoded = true
            r.reply = cur.reply:sub(1, cur.announced * 2)
            r.calinfo = cur.calinfo
            if cur.op == 0x0F then
                st.bpl = hle16(r.reply, 4)
            end
        end
        if cur.op == 0x08 and not cur.calinfo then
            r.bpl, r.indexed = st.bpl, st.indexed
        end
        return r
    end

    return nil
end

-- ------------------------------------------------------------------ decoders --

-- Each decoder fills `t` from `b` (a Tvb over the reassembled bytes) and returns
-- a one-line summary for the Info column.

local function decode_mode_select(t, b)
    if b:len() < 15 then return "truncated MODE SELECT payload" end
    t:add_le(f.ms_resolution, b(2, 2))
    t:add(f.ms_passes, b(4, 1))
    local depth = b(5, 1):uint()
    t:add(f.ms_depth, b(5, 1)):append_text(string.format(" (%s bit)", bitlist(depth, DEPTH_BITS)))
    t:add(f.ms_format, b(6, 1))
    t:add(f.ms_byteorder, b(8, 1)):append_text(b(8, 1):uint() % 2 == 1 and " (Intel)" or " (Motorola)")
    local q = b(9, 1):uint()
    local qt = t:add(f.ms_quality, b(9, 1))
    qt:append_text(string.format(" (%s)", bitlist(q, QUALITY_BITS)))
    qt:add(f.ms_sharpen, b(9, 1))
    qt:add(f.ms_skipshade, b(9, 1))
    qt:add(f.ms_fastir, b(9, 1))
    local af = t:add(f.ms_autofocus, b(10, 1))
    if b(10, 1):uint() == 0x08 then
        af:append_text(" (autofocus pass)")
    end
    t:add(f.ms_halftone, b(12, 1))
    t:add(f.ms_threshold, b(13, 1))
    return string.format("%d dpi, %s, %s bit, %s, quality %s",
        b(2, 2):le_uint(),
        PASSES[b(4, 1):uint()] or string.format("passes 0x%02x", b(4, 1):uint()),
        bitlist(depth, DEPTH_BITS),
        (COLOR_FORMATS[b(6, 1):uint()] or "?"):match("^%w+"),
        bitlist(q, QUALITY_BITS))
        .. (b(10, 1):uint() == 0x08 and " [autofocus]" or "")
end

local function decode_scan_frame(t, b)
    if b:len() < 14 then return "truncated SCAN FRAME payload" end
    t:add_le(f.sf_index, b(4, 2))
    t:add_le(f.sf_x0, b(6, 2))
    t:add_le(f.sf_y0, b(8, 2))
    t:add_le(f.sf_x1, b(10, 2))
    t:add_le(f.sf_y1, b(12, 2))
    local x0, y0 = b(6, 2):le_uint(), b(8, 2):le_uint()
    local x1, y1 = b(10, 2):le_uint(), b(12, 2):le_uint()
    local size = string.format("%.1f x %.1f mm at 1/%d inch",
        mm_frame(x1 - x0), mm_frame(y1 - y0), FRAME_UNITS_PER_INCH)
    t:add(f.sf_size, b(6, 8), size):set_generated()
    return string.format("frame %d: (%d,%d)-(%d,%d), %s",
        b(4, 2):le_uint(), x0, y0, x1, y1, size)
end

local function decode_write(t, b)
    if b:len() < 2 then return "truncated WRITE payload" end
    local sub = b(0, 2):le_uint()
    t:add_le(f.subcmd, b(0, 2))
    if b:len() >= 4 then
        local lt = t:add_le(f.w_length, b(2, 2))
        -- The vendor driver byte-swaps this field on the green entry of 0x13 and
        -- 0x14 in every capture; the device tolerates it.
        if b(2, 2):le_uint() ~= b:len() - 4 and b(2, 2):uint() == b:len() - 4 then
            lt:append_text(" [byte-swapped by the sender]")
        end
    end
    if sub == 0x12 then
        return decode_scan_frame(t, b)
    elseif sub == 0x13 or sub == 0x14 then
        if b:len() < 8 then return "truncated" end
        local filt = b(4, 2):le_uint()
        t:add_le(f.w_filter, b(4, 2)):append_text(string.format(" (%s)", FILTER_MASKS[filt] or "?"))
        t:add_le(f.w_value, b(6, 2))
        return string.format("%s = %d%%",
            FILTER_MASKS[filt] or string.format("filter 0x%02x", filt), b(6, 2):le_uint())
    elseif sub == 0x95 then
        return "request the shading-reference layout"
    end
    return string.format("%s", SUBCMDS[sub] or string.format("subcommand 0x%04X", sub))
end

local function decode_set_gain_offset(t, b)
    if b:len() < 23 then return "truncated SET GAIN OFFSET payload" end
    t:add_le(f.go_exp_r, b(0, 2))
    t:add_le(f.go_exp_g, b(2, 2))
    t:add_le(f.go_exp_b, b(4, 2))
    t:add(f.go_off_r, b(6, 1))
    t:add(f.go_off_g, b(7, 1))
    t:add(f.go_off_b, b(8, 1))
    t:add(f.go_gain_r, b(12, 1))
    t:add(f.go_gain_g, b(13, 1))
    t:add(f.go_gain_b, b(14, 1))
    t:add(f.go_light, b(15, 1))
    t:add(f.go_extra, b(16, 1))
    t:add(f.go_double, b(17, 1))
    t:add_le(f.go_exp_i, b(18, 2))
    t:add(f.go_off_i, b(20, 1))
    t:add(f.go_gain_i, b(22, 1))
    return string.format("exp=%d/%d/%d/%d gain=%d/%d/%d/%d offset=%d/%d/%d/%d light=%d",
        b(0, 2):le_uint(), b(2, 2):le_uint(), b(4, 2):le_uint(), b(18, 2):le_uint(),
        b(12, 1):uint(), b(13, 1):uint(), b(14, 1):uint(), b(22, 1):uint(),
        b(6, 1):uint(), b(7, 1):uint(), b(8, 1):uint(), b(20, 1):uint(),
        b(15, 1):uint())
end

local function decode_slide(t, b)
    if b:len() < 2 then return "truncated SLIDE payload" end
    local action, param = b(0, 1):uint(), b(1, 1):uint()
    t:add(f.sl_action, b(0, 1))
    t:add(f.sl_param, b(1, 1))
    if b:len() >= 4 then t:add(f.sl_byte3, b(3, 1)) end
    local meaning
    if action == 0x00 or action == 0x01 then
        meaning = string.format("move film %s %d steps of 1/%d inch (%.1f mm)",
            action == 0x00 and "forward" or "back", param, OFFSET_STEPS_PER_INCH,
            param * 25.4 / OFFSET_STEPS_PER_INCH)
    elseif action == 0x02 or action == 0x03 then
        meaning = string.format("eject %s", action == 0x02 and "downwards" or "upwards")
    elseif action == 0x04 or action == 0x05 then
        meaning = string.format("%s %d frame(s)", action == 0x04 and "advance" or "rewind", param)
    elseif action == 0x10 then
        meaning = string.format("set focus position to %d", param)
    else
        meaning = "unknown action"
    end
    t:add(f.sl_meaning, b(0, 2), meaning):set_generated()
    return string.format("%s: %s", SLIDE_ACTIONS[action] or
        string.format("action 0x%02x", action), meaning)
end

local function decode_payload(t, op, sub, b)
    if op == 0x15 then return decode_mode_select(t, b) end
    if op == 0x0A then return decode_write(t, b) end
    if op == 0xDC then return decode_set_gain_offset(t, b) end
    if op == 0xD1 then return decode_slide(t, b) end
    return string.format("%d payload bytes", b:len())
end

local function decode_inquiry(t, b)
    if b:len() < 184 then return string.format("INQUIRY reply truncated to %d bytes", b:len()) end
    t:add(f.iq_vendor, b(8, 8), txt(b(8, 8)))
    t:add(f.iq_product, b(16, 16), txt(b(16, 16)))
    t:add(f.iq_revision, b(32, 4), txt(b(32, 4)))
    t:add_le(f.iq_res_x, b(36, 2))
    t:add_le(f.iq_res_y, b(38, 2))
    local w, h = b(40, 2):le_uint(), b(42, 2):le_uint()
    t:add_le(f.iq_scan_w, b(40, 2)):append_text(string.format(" (%.1f mm)", mm_frame(w)))
    t:add_le(f.iq_scan_h, b(42, 2)):append_text(string.format(" (%.1f mm)", mm_frame(h)))
    t:add(f.iq_filters, b(44, 1)):append_text(string.format(" (%s)", bitlist(b(44, 1):uint(), FILTER_MASKS)))
    t:add(f.iq_depths, b(45, 1)):append_text(string.format(" (%s bit)", bitlist(b(45, 1):uint(), DEPTH_BITS)))
    t:add(f.iq_formats, b(46, 1)):append_text(string.format(" (%s)", bitlist(b(46, 1):uint(), FORMAT_BITS)))
    -- The low three bits of byte 49 count scan speeds; the rest are flags.
    local caps = b(49, 1):uint()
    t:add(f.iq_caps, b(49, 1)):append_text(string.format(" (%s, %d scan speeds)",
        bitlist(caps - caps % 8, CAPABILITY_BITS), caps % 8))
    t:add(f.iq_optional, b(50, 1)):append_text(string.format(" (%s)", bitlist(b(50, 1):uint(), OPTIONAL_BITS)))
    t:add_le(f.iq_preview, b(54, 2))
    t:add(f.iq_firmware, b(96, 4), txt(b(96, 4)))
    t:add_le(f.iq_max_exp, b(104, 2))
    t:add_le(f.iq_min_exp, b(106, 2))
    t:add_le(f.iq_model, b(116, 2))
    t:add(f.iq_production, b(120, 4), txt(b(120, 4)))
    t:add(f.iq_timestamp, b(124, 20), txt(b(124, 20)))
    t:add(f.iq_signature, b(144, 40), txt(b(144, 40)))
    local model = b(116, 2):le_uint()
    return string.format("%s '%s' rev %s, model 0x%04x %s, %dx%d dpi, bed %dx%d (%.1f x %.1f mm)",
        txt(b(8, 8)), txt(b(16, 16)), txt(b(32, 4)), model,
        MODELS[model] or "unknown model", b(36, 2):le_uint(), b(38, 2):le_uint(),
        w, h, mm_frame(w), mm_frame(h))
end

local function decode_read_state(t, b)
    if b:len() < 7 then return string.format("READ STATE reply truncated to %d bytes", b:len()) end
    t:add(f.rs_frame, b(2, 1))
    t:add(f.rs_focus, b(3, 1))
    t:add(f.rs_focus_max, b(4, 1))
    local flags = b(6, 1):uint()
    local ft = t:add(f.rs_flags, b(6, 1))
    ft:add(f.rs_medium, b(6, 1))
    ft:add(f.rs_strip, b(6, 1))
    if b:len() >= 8 then t:add(f.rs_byte7, b(7, 1)) end
    local what = {}
    if flags % 2 == 1 then what[#what + 1] = "medium present" end
    if flags % 8 >= 4 then what[#what + 1] = "filmstrip carrier" end
    return string.format("frame %d, focus %d of %d, flags 0x%02x%s",
        b(2, 1):uint(), b(3, 1):uint(), b(4, 1):uint(), flags,
        #what > 0 and (" (" .. table.concat(what, ", ") .. ")") or "")
end

local function decode_get_parameters(t, b)
    if b:len() < 16 then return string.format("GET PARAMETERS reply truncated to %d bytes", b:len()) end
    t:add_le(f.gp_width, b(0, 2))
    t:add_le(f.gp_lines, b(2, 2))
    t:add_le(f.gp_bpl, b(4, 2))
    t:add_le(f.gp_avail, b(14, 2))
    return string.format("width %d, %d lines, %d bytes per line, %d available",
        b(0, 2):le_uint(), b(2, 2):le_uint(), b(4, 2):le_uint(), b(14, 2):le_uint())
end

local function decode_sense(t, b)
    if b:len() < 14 then return string.format("sense data truncated to %d bytes", b:len()) end
    -- The top nibble of byte 2 carries FILEMARK/EOM/ILI, not the sense key.
    local key = b(2, 1):uint() % 16
    local asc, ascq = b(12, 1):uint(), b(13, 1):uint()
    t:add(f.sn_key, b(2, 1), key)
    t:add(f.sn_asc, b(12, 1))
    t:add(f.sn_ascq, b(13, 1))
    local meaning = SENSE_MEANING[string.format("%02x/%02x/%02x", key, asc, ascq)]
    if meaning then t:add(f.sn_meaning, b(2, 12), meaning):set_generated() end
    return string.format("key 0x%02x %s, asc/ascq %02x/%02x%s", key,
        SENSE_KEYS[key] or "?", asc, ascq, meaning and (" -- " .. meaning) or "")
end

local function decode_read_gain_offset(t, b)
    if b:len() < 103 then return string.format("gain/offset reply truncated to %d bytes", b:len()) end
    t:add_le(f.go_sat_r, b(54, 2))
    t:add_le(f.go_sat_g, b(56, 2))
    t:add_le(f.go_sat_b, b(58, 2))
    t:add_le(f.go_exp_r, b(60, 2))
    t:add_le(f.go_exp_g, b(62, 2))
    t:add_le(f.go_exp_b, b(64, 2))
    t:add(f.go_off_r, b(66, 1))
    t:add(f.go_off_g, b(67, 1))
    t:add(f.go_off_b, b(68, 1))
    t:add(f.go_gain_r, b(72, 1))
    t:add(f.go_gain_g, b(73, 1))
    t:add(f.go_gain_b, b(74, 1))
    t:add(f.go_light, b(75, 1))
    t:add_le(f.go_exp_i, b(98, 2))
    t:add(f.go_off_i, b(100, 1))
    t:add(f.go_gain_i, b(102, 1))
    return string.format("exp=%d/%d/%d/%d gain=%d/%d/%d/%d offset=%d/%d/%d/%d light=%d saturation=%d/%d/%d",
        b(60, 2):le_uint(), b(62, 2):le_uint(), b(64, 2):le_uint(), b(98, 2):le_uint(),
        b(72, 1):uint(), b(73, 1):uint(), b(74, 1):uint(), b(102, 1):uint(),
        b(66, 1):uint(), b(67, 1):uint(), b(68, 1):uint(), b(100, 1):uint(),
        b(75, 1):uint(),
        b(54, 2):le_uint(), b(56, 2):le_uint(), b(58, 2):le_uint())
end

local function decode_calibration_info(t, b)
    if b:len() < 8 then return string.format("calibration info truncated to %d bytes", b:len()) end
    local n, size = b(4, 1):uint(), b(5, 1):uint()
    t:add(f.ci_entries, b(4, 1))
    t:add(f.ci_entrysize, b(5, 1))
    local total = 0
    for k = 0, n - 1 do
        local off = 8 + size * k
        if off + 6 > b:len() then break end
        local et = t:add(b(off, size), string.format("Entry %d", k + 1))
        et:add(f.ci_type, b(off, 1))
        et:add(f.ci_lines, b(off + 3, 1))
        et:add_le(f.ci_pixels, b(off + 4, 2))
        et:append_text(string.format(": type 0x%02x, %d lines of %d pixels",
            b(off, 1):uint(), b(off + 3, 1):uint(), b(off + 4, 2):le_uint()))
        total = total + b(off + 3, 1):uint()
    end
    return string.format("%d entries of %d bytes, %d shading lines in total", n, size, total)
end

local function decode_reply(t, r, b)
    local op = r.op
    if op == 0x12 then return decode_inquiry(t, b) end
    if op == 0xDD then return decode_read_state(t, b) end
    if op == 0x0F then return decode_get_parameters(t, b) end
    if op == 0x03 then return decode_sense(t, b) end
    if op == 0xD7 then return decode_read_gain_offset(t, b) end
    if op == 0x08 and r.calinfo then return decode_calibration_info(t, b) end
    return string.format("%d reply bytes", b:len())
end

-- ------------------------------------------------------------------- display --

local function spaced(hex)
    return (hex:gsub("(%x%x)", "%1 "):gsub("%s+$", ""))
end

local function display(tvb, pinfo, tree, r)
    local frag, resp, cap = firange(f_frag), firange(f_resp), firange(f_capdata)
    local data = frag or resp or cap
    local zr = tvb(0, 0)
    local name = opname(r.op, r.sub)
    local root = tree:add(p_pieusb, data or zr, "PIE scanner")
    local info

    if r.txn then
        root:add(f.txn, zr, r.txn):set_generated()
        if r.op then
            root:add(f.opcode, zr, r.op):set_generated()
            root:add(f.command, zr, name):set_generated()
        end
        if r.sub then root:add(f.subcmd, zr, r.sub):set_generated() end
    end
    if r.port then
        local wv = firange(f_wval)
        if wv then root:add_le(f.port, wv) end
    end

    local val = frag and frag:range(0, 1):uint() or nil
    local prefix = r.txn and string.format("txn %d ", r.txn) or ""

    if r.kind == "wakeup" then
        root:add(f.regval, frag)
        root:add(f.wakeup_idx, zr, r.idx):set_generated()
        info = string.format("IEEE1284 wakeup byte %d/%d = 0x%02x", r.idx, #WAKEUP, val)

    elseif r.kind == "ieee" then
        root:add(f.ieee, frag)
        info = string.format("IEEE1284 %s", IEEE_CMDS[r.ieee] or string.format("command 0x%02x", r.ieee))
        if r.txn then info = info .. string.format(" -- transaction %d begins", r.txn) end

    elseif r.kind == "ctrl" then
        local it = root:add(f.regval, frag)
        it:add(f.nstrobe, frag)
        it:add(f.ninit, frag)
        local bits = {}
        if val % 2 == 1 then bits[#bits + 1] = "nSTROBE" end
        if val % 8 >= 4 then bits[#bits + 1] = "nINIT" end
        info = string.format("parallel control = 0x%02x (%s)", val,
            #bits > 0 and table.concat(bits, " ") or "idle")

    elseif r.kind == "data" then
        root:add(f.regval, frag)
        info = string.format("parallel data = 0x%02x", val)

    elseif r.kind == "cdb" then
        root:add(f.regval, frag)
        root:add(f.cdb_idx, zr, r.idx):set_generated()
        if r.cdb then
            local b = ByteArray.new(r.cdb):tvb("PIEUSB CDB")
            local ct = root:add(f.cdb, b(0, 6))
            ct:add(f.opcode, b(0, 1))
            ct:add(f.cdb_len, b(3, 2))
            ct:append_text(string.format(" -- %s, length field %d", name, b(3, 2):uint()))
            if not OPCODES[r.op] then ct:add_proto_expert_info(e_unknown) end
            info = string.format("%s%s: CDB %s, length field %d", prefix, name,
                spaced(r.cdb), b(3, 2):uint())
        elseif r.idx == 0 then
            info = string.format("%sCDB byte 0 = 0x%02x (%s)", prefix, val,
                OPCODES[val] or "unknown opcode")
        else
            info = string.format("%sCDB byte %d = 0x%02x", prefix, r.idx, val)
        end

    elseif r.kind == "payload" then
        root:add(f.regval, frag)
        root:add(f.payload_idx, zr, r.idx):set_generated()
        if r.payload then
            local b = ByteArray.new(r.payload):tvb("PIEUSB payload")
            local pt = root:add(f.payload, b(0, b:len()))
            local summary = decode_payload(pt, r.op, r.sub, b)
            root:add(f.summary, zr, summary):set_generated()
            info = string.format("%s%s: %s", prefix, name, summary)
        else
            info = string.format("%s%s: payload byte %d = 0x%02x", prefix, name, r.idx, val)
        end

    elseif r.kind == "announce" then
        if frag and frag:len() >= 8 then root:add_le(f.announce, frag:range(4, 4)) end
        info = string.format("%s%s: announcing %s reply bytes", prefix, name,
            r.size and tostring(r.size) or "?")

    elseif r.kind == "status" then
        if not resp then return end
        local s = resp:range(0, 1):uint()
        root:add(f.status, resp:range(0, 1))
        if s == 0x02 then
            root:add_proto_expert_info(e_check)
        elseif s == 0x88 or s == 0xFF then
            root:add_proto_expert_info(e_fail)
        end
        info = string.format("%s%s: status %s", prefix, name,
            STATUSES[s] or string.format("0x%02x", s))

    elseif r.kind == "bulk" then
        root:add(f.reply_len, zr, r.len):set_generated()
        if r.off then root:add(f.reply_off, zr, r.off):set_generated() end
        if r.total then root:add(f.reply_total, zr, r.total):set_generated() end
        if r.missing then
            root:add(f.uncaptured, zr, r.missing):set_generated()
            root:add_proto_expert_info(e_short)
        end
        if r.reply then
            local b
            if r.off == 0 and cap and cap:len() * 2 >= #r.reply then
                b = cap:tvb()
            else
                b = ByteArray.new(r.reply):tvb("PIEUSB reply")
            end
            local rt = root:add(f.reply, b(0, math.floor(#r.reply / 2)))
            local summary = decode_reply(rt, r, b)
            root:add(f.summary, zr, summary):set_generated()
            if r.op == 0x03 and b:len() >= 14 and b(2, 1):uint() % 16 ~= 0 then
                root:add_proto_expert_info(e_sense)
            end
            info = string.format("%s%s reply: %s", prefix, name, summary)
        else
            info = string.format("%s%s: %d reply bytes at offset %d", prefix, name,
                r.len, r.off or 0)
        end
        -- Image data: the two-byte channel tag that prefixes every line in INDEX
        -- colour format. Lines arrive one per channel in rotation, not in planes.
        if r.bpl and r.indexed and cap then
            local stride = r.bpl + 2
            local k = math.ceil(r.off / stride)
            local names = {}
            while #names < MAX_TAGS do
                local pos = k * stride - r.off
                if pos < 0 or pos + 2 > cap:len() then break end
                local tag = cap:range(pos, 2):uint()
                if not LINE_TAGS[tag] then break end
                root:add(f.img_tag, cap:range(pos, 2)):append_text(
                    string.format("  (line %d)", k))
                names[#names + 1] = LINE_TAGS[tag]
                k = k + 1
            end
            if #names > 0 then
                local rotation = table.concat(names)
                root:add(f.img_rotation, zr, rotation):set_generated()
                info = info .. ", lines " .. rotation .. (#names >= MAX_TAGS and "..." or "")
            end
        end
        if r.zeros then
            info = info .. string.format(", %d zero bytes so far (pixels the shading reference covers)", r.zeros)
        end

    else
        return
    end

    pinfo.cols.protocol = "PIEUSB"
    pinfo.cols.info = info
    root:set_text("PIE scanner  " .. info)
end

p_pieusb.dissector = function(tvb, pinfo, tree)
    local r
    if pinfo.visited then
        r = recs[pinfo.number]
    else
        r = analyze(pinfo)
        recs[pinfo.number] = r
    end
    if not r then return end
    local ok, err = pcall(display, tvb, pinfo, tree, r)
    if not ok then
        local root = tree:add(p_pieusb, tvb(0, 0), "PIE scanner")
        root:add_proto_expert_info(e_error, tostring(err))
    end
end

register_postdissector(p_pieusb)
