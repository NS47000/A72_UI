"""API for JBD 4020 control board.

Implements a Python wrapper for the USB-UART protocol to the JBD 4020 control
board.  UART data protocol outlined in JBD Comm_protocol_4020_V2.2.4 document
"""
from enum import Enum
import inspect
import logging
import os
import time
import cv2
import numpy as np
import serial
import serial.tools.list_ports
from dataclasses import dataclass, fields
from typing import Union
CMD_SUFFIX = b"\x4a\x42\x44"  # b"JBD" at the end of each sent msg
# TODO: does this take a long with a panel connected?
RESET_DELAY = 15  # after a reset the board takes a while to come up
PORT_DESCRIPTION_FILTER = "Silicon Labs CP210x USB to UART Bridge"
# Enumerate the known Control Board FW versions
CB_FW_VERSIONS = {
  "V1.12.06": 1,
  "V1.13.08": 2,
  "V1.14.08": 3,
  "V1.14.15": 4,
  "V1.14.21": 5,
  "V1.14.G0": 6,
  "V1.14.G3": 7,
  "V1.14.G4": 8,
  "V1.14.G7": 9
}

def find_port():
  ports = serial.tools.list_ports.comports()
  for p in ports:
    if PORT_DESCRIPTION_FILTER in p.description:
      return p.device
  else:
    raise RuntimeError("Unable to locate matching device.")

class GammaPath(Enum):
  """Defines paths to predefined gamma LUT tables.

     Used with pass_thru method.
  """
  current_file_directory = os.path.dirname(os.path.abspath(__file__))
  gs_1_0_lreg_1_0 = os.path.join(current_file_directory,
                                 "gamma1.0_2.csv")
  gs_2_2_lreg_2_2 = os.path.join(current_file_directory,
                                 "gamma2.2_lreg_gs_FW1.13.08.csv")

class GammaSetting(Enum):
  """Defines byte to set greyscale and DBV gamma.

     Used with built in Control Board method.
  """
  gs_2_2_lreg_1_0 = b"\x00"
  gs_1_0_lreg_1_0 = b"\x01"
  gs_2_2_lreg_2_2 = b"\x02"
  default_flash = b"\x03"

class JbdControlMsg(Enum):
  """Control messages from JBD Comm_protocol_4020_V2.2.3.
  """
  initialize_panel = b"\x86"
  get_firmware = b"\xc0"
  system_reset = b"\x84"
  control_panel_reset = b"\x55"
  display_enable = b"\x68"
  i2c_interface_enable = b"\x51"
  read_temp_sensor = b"\x3f"
  # display controls
  display_mono_image = b"\x34"
  display_color_image = b"\x35"
  write_color_image_rle = b"\xc2\x01" #used when writing images from the JBD GUI
  set_hdmi = b"\xc4" #use to enable / disable hdmi
  draw_rectangle = b"\x80"
  clear_display = b"\x81"
  write_creg = b"\x24"
  write_lreg = b"\x23"
  # orientation controls
  image_offset = b"\x5b"  # "Offset Setting??"
  set_h_mirror = b"\x58"
  set_v_mirror = b"\x59"
  load_gamma_data = b"\x65"
  gamma_on_off = b"\x5f"
  set_display_resolution = b"\x67"
  dither_enable = b"\x69"
  random_scan_control = b"\x6a"
  # low level operations
  write_register_all = b"\x01"
  write_register_red = b"\xb0"
  write_register_green = b"\xb1"
  write_register_blue = b"\xb2"
  read_register = b"\x03"
  write_efuse = b"\x04"
  read_efuse = b"\x05"
  write_die_id = b"\x06"
  read_die_id = b"\x07"
  # TF card
  write_tf_card = b"\x82"
  tf_card_operation = b"\x85"
  write_tf_image_to_display = b"\x83"

class JbdPanel(Enum):
  """Defines panel selection option byte."""
  all = b"\x07"
  red = b"\x01"
  green = b"\x02"
  blue = b"\x04"

class JbdPaneResolutionIdx(Enum):
  res_640x480 = b"\x00"
  res_660x504 = b"\x01"

# @dataclass
# class JbdI2cAddr:
#   """Structure organizing a value associated with a I2C Address"""
#   default_0x58: str = None
#   r_0x59: str = None
#   g_0x5A: str = None
#   b_0x5B: str = None

@dataclass
class JbdReadResponse:
  """Structure organizing a read response associated with a connector on the
  daughter board.
  """
  # r_b2b: JbdI2cAddr = None
  # g_b2b: JbdI2cAddr = None
  # b_b2b: JbdI2cAddr = None
  r_b2b_default: str = None
  r_b2b_r_0x59: str = None
  r_b2b_g_0x5A: str = None
  r_b2b_b_0x5B: str = None
  g_b2b_default: str = None
  g_b2b_r_0x59: str = None
  g_b2b_g_0x5A: str = None
  g_b2b_b_0x5B: str = None
  b_b2b_default: str = None
  b_b2b_r_0x59: str = None
  b_b2b_g_0x5A: str = None
  b_b2b_b_0x5B: str = None

@dataclass
class JbdMirrorOffsetConfig:
  """Structure organizing Mirror (flip) and offset config for SRAM and VO."""
  sram_mirror_flip: int = None        #value in SRAM config register 0x03030004
  sram_lr_mirror: bool = None
  sram_ud_mirror: bool = None         #up-down mirror aka flip
  sram_offset: int = None             #value in SRAM config register 0x03030008
  sram_offset_en: bool = None
  sram_offset_x: int = None
  sram_offset_y: int = None
  vo: int = None                      #value in SRAM config register 0x0300015C
  vo_ud_mirror: bool = None
  vo_lr_mirror: bool = None
  vo_offset_en: bool = None
  vo_offset_y: int = None
  vo_offset_x: int = None

  def recalculate(self):
    """Recalculates the offset and mirror parameters from register data.

    Calculates L-R & U-D mirror enable, offset enable, and X & Y offset
    values from the register values

    Args:
     None

    Returns:
      None

    Raises:
      None
    """
    if (self.sram_mirror_flip is not None and
        self.sram_offset is not None and
        self.vo is not None):
      self.sram_lr_mirror = bool(0x1 & self.sram_mirror_flip)
      self.sram_ud_mirror = bool((0x2 & self.sram_mirror_flip) >> 1)

      self.sram_offset_en = bool(0x1 & self.sram_offset)
      self.sram_offset_x = (0x1F0 & self.sram_offset) >> 4
      self.sram_offset_y = (0x1F000 & self.sram_offset) >> 12

      self.vo_ud_mirror = bool((0x80000000 & self.vo) >> 31)
      self.vo_lr_mirror = bool((0x40000000 & self.vo) >> 30)
      self.vo_offset_en = bool((0x20000000 & self.vo) >> 29)
      self.vo_offset_y = (0x0FFF0000 & self.vo) >> 16
      self.vo_offset_x = 0x00000FFF & self.vo
      if self.sram_offset_en != self.vo_offset_en:
        self.logger.warning("SRAM offset enable = %s & VO offset enable = %s "
                            "do not match",
                            bool(self.sram_offset_en),
                            bool(self.vo_offset_en))
      if self.sram_lr_mirror != self.vo_lr_mirror:
        self.logger.warning("SRAM LR mirror enable = %s & VO LR mirror enable ="
                            " %s do not match",
                            bool(self.sram_lr_mirror),
                            bool(self.vo_lr_mirror))
      if self.sram_ud_mirror != self.vo_ud_mirror:
        self.logger.warning("SRAM UD mirror enable = %s & VO UD mirror enable ="
                            " %s do not match",
                            bool(self.sram_ud_mirror),
                            bool(self.vo_ud_mirror))
      if self.sram_offset_x != self.vo_offset_x:
        self.logger.warning("SRAM offset x = %s & VO offset x = %s "
                            "do not match",
                            self.sram_offset_x,
                            self.vo_offset_x)
      if self.sram_offset_y != self.vo_offset_y:
        self.logger.warning("SRAM offset y = %s & VO offset y = %s "
                            " do not match",
                            self.sram_offset_y,
                            self.vo_offset_y)

class RefreshRate(Enum):
  refresh_rate_30 = [0x20, 0x49, 0x00, 0x3F]
  refresh_rate_37_5 = [0x20, 0x47, 0x00, 0x3F]
  refresh_rate_50 = [0x20, 0x45, 0x00, 0x3F]
  refresh_rate_60 = [0x20, 0x44, 0x00, 0x3F]
  refresh_rate_75 = [0x22, 0x43, 0x00, 0x3F]

class MipiRefreshConfig(Enum):
  mipi_refresh_75 = [0x00, 60, 0x0f, 0x86]
  mipi_refresh_other = [0x00, 0x60, 0x0f, 80]

# convert the resolutions above to a tuple
_PANEL_RESOLUTIONS = {JbdPaneResolutionIdx.res_640x480: (640, 480),
                      JbdPaneResolutionIdx.res_660x504: (660, 504)}
class Jbd4020SerialUart(serial.Serial):
  """Serial Uart for JBD control board."""

  def __init__(self, *args, **kwargs):
    """Create serial instance with specified port."""
    self.logger = logging.getLogger()
    self.alias = kwargs.get("alias", "Jbd4020SUart")
    if "port" in kwargs:
      kwargs["port"] = kwargs.pop("port")
    elif len(args) > 0:
      kwargs["port"] = args[0]
    if not kwargs.get("port") or kwargs["port"] is None:
      kwargs["port"] = find_port()
    self.com_port = kwargs["port"]
    kwargs["baudrate"] = kwargs.pop("baudrate", 921600)
    kwargs["bytesize"] = kwargs.pop("bytesize", 8)  # aka data bits
    kwargs["stopbits"] = kwargs.pop("stopbits", serial.STOPBITS_ONE)
    kwargs["parity"] = kwargs.pop("parity", serial.PARITY_NONE)
    kwargs["timeout"] = kwargs.pop("timeout", 3)
    kwargs["write_timeout"] = kwargs.pop("write_timeout", 15)  # default is None
    kwargs["inter_byte_timeout"] = kwargs.pop("inter_byte_timeout", 3)
    self.read_timeout = kwargs.pop("read_timeout", 5 )
    super().__init__(**kwargs)

  def send(self, msg: bytes):
    """Write the msg to the serial/uart port.

    Args:
      msg: bytes to be sent to the serial port

    Returns:
      None

    Raises:
      None
    """
    caller_func = inspect.stack()[1][3]  # this is here for debugging
    if caller_func == "query":
      caller_func = inspect.stack()[2][3]
    if len(msg) == 12:
      self.logger.debug("tx: %s - %s",
                        "".join("{:02x}".format(x) for x in msg),
                        caller_func)
    else:
      # don"t print huge messages
      self.logger.debug("tx: %s ... %s - %s",
                        "".join("{:02x}".format(x) for x in msg[:12]),
                        "".join("{:02x}".format(x) for x in msg[-6:]),
                        caller_func)
    this_block = msg[0:12]
    # make sure the UART is clear of any left over garbage in the buffers
    self.reset_input_buffer()
    self.reset_output_buffer()
    self.write(bytearray(this_block))
    # split the data_flow in to 3000byte blocks and send an ACK each time
    # this was reverse engineered using wire shark and the CTRL-BRD app.
    # TODO(b/267195219): see if this can be done with longer timeouts.
    for x in range(12, len(msg), 3000):
      if x + 3000 > len(msg):
        this_block = msg[x:]  # TODO: if the last block is len 1 it might cause a type error..
      else:
        this_block = msg[x:x+3000]
      self.write(bytearray(this_block))
      self.write(b"")  # write ACK? this is not documented
      # # below was done to get an error, send/query will have to be refactored to incorporate this.
      # r = self.read_all()
      # if "Received" in r.decode(errors="backslashreplace"):
      #   print(r)
      #   raise RuntimeError("Error detected while writing data flow")

  def query(self, msg: bytes, response_delay=0) -> bytearray:
    """Send a message and get the response / ACK.

    Args:
      msg: bytes to be sent to the serial port

    Returns:
      A bytearray of values read back from the serial port in response to the
      sent message.

    Raises:
      None
    """
    caller_func = inspect.stack()[1][3]
    self.send(msg)
    time.sleep(response_delay)
    read_timeout = time.time() + self.read_timeout
    #print(self.in_waiting)
    while not self.in_waiting:
      if time.time() > read_timeout:
        raise TimeoutError(f"No response from device - {caller_func}")
      time.sleep(0.01)  # don"t pin the CPU
    buffer = bytearray()
    while self.in_waiting:
      time.sleep(0.1)  # some devices pause between lines...
      buffer += self.read_all()
    if buffer[0:2].decode(errors="backslashreplace") == "Rx":
      self.logger.warning("Error from %s: %s - %s",
                          self.alias,
                          buffer[:12].decode(errors="backslashreplace").strip(),
                          caller_func)
    if buffer[0:1] != msg[0:1]:  # use slices not index for python reasons
      self.logger.warning("response does not start with message byte"
                          "%s vs %s - %s", msg[0:1], buffer[0:1], caller_func)
    if buffer[9:12] != msg[9:12]:
      self.logger.warning("response does not end with b'JBD' - %s", caller_func)
    #if len(buffer) <= 18:
    # increased to 73.  Base message 13 + 5 bytes * 12 possible indices
    # (12 indices are outlined in Appendix 1 of Comm Protocol V2.2.4)
    if len(buffer) <= 73:
      self.logger.debug("rx: %s - %s",
                        "".join("{:02x}".format(x) for x in buffer),
                        caller_func)
    else:
      # truncate large messages
      self.logger.warning("Truncated message.  rx len: %d, info?: %s",
                        len(buffer),
                        buffer[-150:].decode(errors='backslashreplace').strip())
      #print(f"rx len: {len(buffer)}, info?:"
      #      f"{buffer[-150:].decode(errors='backslashreplace').strip()}")
      self.logger.debug("rx: %s ... %s - %s",
                        "".join("{:02x}".format(x) for x in buffer[:12]),
                        "".join("{:02x}".format(x) for x in buffer[-10:]),
                        caller_func)
    return buffer


class Jbd4020CtrlBrd(Jbd4020SerialUart):
  """JBD 4020 Control Board API."""

  def __init__(self, *args, **kwargs):
    """Initializes an instance of the JBD 4020 control board interface.

    Args:
      channel_vdd:
        a power supply channel object instance with a enable/disable methods.
      channel_avdd:
        a power supply channel object instance with a enable/disable methods.
      channel_avee:
        a power supply channel object instance with a enable/disable methods.
      resolution:
        specify a resolution during init (None to leave it as is).
      write_gamma:
        bool, true = write the gamma tables during init.
      init_only:
        bool, true = skip sequencing of externally provided power rails.
      use_pass_thru:
        bool, true = use the write_register method to write the init sequence
        line by line.  false = use the init sequence built into the control
        board FW.
      panel:
        one of JbdPanel.all, JbdPanel.red, JbdPanel.green, JbdPanel.blue
    """
    self.logger = logging.getLogger()
    self.init_only = kwargs.pop("init_only", None)
    self.use_pass_thru = kwargs.pop("use_pass_thru", None)
    self.resolution_idx = kwargs.pop("resolution", None)
    self._resolution = None
    self.power_channel_vdd = kwargs.pop("power_channel_vdd", None)
    self.power_channel_avdd = kwargs.pop("power_channel_avdd", None)
    self.power_channel_avee = kwargs.pop("power_channel_avee", None)
    self.write_gamma = kwargs.pop("write_gamma", None)
    self.panel = kwargs.pop("panel", JbdPanel.all)
    super().__init__(*args, **kwargs)
    self.cb_fw_version = self.check_control_board_fw_version()

  def _get_padding(self, msg):
    """Add padding to the command message.

       The command message has to be padded in the middle with b'x00'
       to be 12 bytes long.
    """
    return b"\x00" * (12 - (len(msg) + len(CMD_SUFFIX)))

  def _bytes_to_hexstring(self, data):
    return "".join("{:02x}".format(x) for x in data)

  def _bytes_to_int(self, data):
    return int(self._bytes_to_hexstring(data), 16)

  def _build_msg(self, msg):
    return bytearray(msg) + self._get_padding(msg) + CMD_SUFFIX

  def _parse_data_flow(self, data):
    """Sometimes we have a data_flow after the command."""
    if len(data) == 12:
      # no data_flow, return the bytes within. TODO: does this work for all CMDS?
      return data[1:9].decode(errors="backslashreplace")
    flow_len = self._bytes_to_int(data[1:5])
    data_flow = data[12:12+flow_len]
    chk = data[-1].to_bytes(1, "big")
    cchk = self._calc_xor_checksum(data[12:-1])
    if cchk != chk:
      self.logger.warning("data_flow checksum does not match. %s <> %s",
                          cchk, chk)
    return data_flow

  @staticmethod
  def _calc_xor_checksum(data):
    chk = 0
    for b in data:
      chk ^= b
    return chk.to_bytes(1, "big")

  @ staticmethod
  def _calc_temp_from_value(value):
    return value * 0.2028 - 64.052

  def system_reset(self):
    """soft reset the control board and the panel.

    Args:
      None

    Returns:
      bytearray response from the control board

    Raises:
      None
    """

    ctrl_msg = JbdControlMsg.system_reset.value
    return self.query(self._build_msg(ctrl_msg), response_delay=4)

  def set_reset_pin(self, value):
    """Set the reset pin high or low.

    Args:
      value:
        int, 1 = set pin high. 0 = set pin low.

    Returns:
      bytearray response from the control board

    Raises:
      None
    """

    ctrl_msg = JbdControlMsg.control_panel_reset.value + bytearray([value])
    return self.query(self._build_msg(ctrl_msg))

  def set_hdmi(self, enable=False):
    """Set HDMI enabled or disabled.

    Args:
      value:
        bool, True = enable HDMI, False = disable HDMI

    Returns:
      bytearray response from the control board

    Raises:
      None
    """

    if enable:
      value = 1
    else:
      value = 0
    ctrl_msg = JbdControlMsg.set_hdmi.value + bytearray([value])
    return self.query(self._build_msg(ctrl_msg))

  def power_on_sequence(self, **kwargs):
    """Power on the panel per the device spec.

    The panel has a power on sequence that requires we power on two channels,
    init the panel then power on the third.

    Args:
      **kwargs: init_only, channel_vdd, channel_avdd, channel_avee, resolution,
      write_gamma

    Where:
      init_only:
        bool, set to true if the power rails are already turned on, this will
        skip turning on the power.
      write_gamma:
        bool, set to true to load gamma tables immediately following the
        initialization register write sequence
      channel_vdd:
        power supply object, with enable/disable methods.
      channel_avdd:
        power supply object, with enable/disable methods.
      channel_avee:
        power supply object, with enable/disable methods.
      resolution:
        JbdPaneResolutionIdx, to define the panel resolution.

    Returns:
      None

    Raises:
      RuntimeError: If not(init_only) and power supplies are not defined.
    """

    self.init_only = kwargs.pop("init_only", self.init_only)
    self.use_pass_thru = kwargs.pop("use_pass_thru", self.use_pass_thru)
    self.power_channel_vdd = kwargs.pop("power_channel_vdd",
                                        self.power_channel_vdd)
    self.power_channel_avdd = kwargs.pop("power_channel_avdd",
                                         self.power_channel_avdd)
    self.power_channel_avee = kwargs.pop("power_channel_avee",
                                         self.power_channel_avee)
    self.resolution_idx = kwargs.pop("resolution", self.resolution_idx)
    self.write_gamma = kwargs.pop("write_gamma", self.write_gamma)
    self.panel = kwargs.pop("panel", self.panel)
    if not(self.init_only):
      if not all([self.power_channel_vdd, self.power_channel_avdd,
                  self.power_channel_avee]):
        raise RuntimeError("No power supply channels defined.")
      self.power_channel_vdd.enable()
      time.sleep(0.2)
      self.power_channel_avdd.enable()
      time.sleep(1)
    if self.use_pass_thru:
      self.initialize_panel_pass_thru(self.panel)
      self.set_display_enable(False, JbdPanel.all)
      self.set_display_enable(True, self.panel)
    else:
      self.initialize_panel(self.resolution_idx)
    self.clear_screen()
    time.sleep(1)
    if self.write_gamma:
      self.logger.info("writing gamma tables")
      self.write_gamma_tables()
      self.logger.info("writing gamma tables completed")
    if not(self.init_only):
      time.sleep(0.5)
      self.power_channel_avee.enable()

  def power_off_sequence(self):
    """Power off the power rails.

    Args:
      None

    Returns:
      None

    Raises:
      RuntimeError: if power supply channels are not defined.
    """
    if not all([self.power_channel_vdd, self.power_channel_avdd,
                self.power_channel_avee]):
      raise RuntimeError("No power supply channels defined, did you mean "
                         "<station>.power_off_sequence?")
    self.power_channel_avee.disable()
    time.sleep(0.1)
    self.power_channel_avdd.disable()
    time.sleep(0.1)
    self.power_channel_vdd.disable()

  def initialize_panel_pass_thru(self, panel: JbdPanel = JbdPanel.all):
    """Intialize the panel using the latest known-good init sequence.

    Init sequence is written as pass_thru, meaning direct line by line
    register writes rather than the built in init method in JBD Contol Board FW.
    Currently uses the JBD Init Code 1.6.4, modified to assume no flash memory
    is present.
    Color specific values (for example setting mipi_rgb_sel in register
    0x03040000 are directed to the appropriate channel, if that channel or
    JbdPanel.all are specified)

    Args:
      panel:
        JbdPanel, set the panel (red, green, blue or all) panels to be
        initialized

    Returns:
      None

    Raises:
      None
    """
    self.logger.info("Initializing panel: %s, with pass thru method", panel)
    ## SRAM Initialization
    # Switch off PWM CLK
    self.set_register("0x03030300", "0x00000010", panel)
    # Panel Initialization
    self.set_register("0x03030144", "0x40000000", panel)
    # MUTE set max resolution
    self.set_register("0x030001C8", "0x0029F1FF", panel)
    # Background gray value set 0
    self.set_register("0x030001B8", "0x00000000", panel)
    # MUTE enable
    self.set_register("0x030001C4", "0x00000001", panel)
    # Display background color mode
    self.set_register("0x03000180", "0x2003d01B", panel)
    # Display port enable
    self.set_register("0x03030100", "0x80000011", panel)
    self.set_register("0x03040008", "0x00000001", panel)
    # wait for 20ms
    time.sleep(0.02)
    # XDP reset
    self.set_register("0x0100003C", "0x20021F3F", panel)
    # XDP reset release
    self.set_register("0x0100003C", "0x2002003F", panel)
    # Clear MIPI interruption
    self.set_register("0x02006720", "0xFFFFFFFF", panel)
    self.set_register("0x02006724", "0xFFFFFFFF", panel)
    self.set_register("0x02006728", "0xFFFFFFFF", panel)
    # Error count clear
    self.set_register("0x02006740", "0xFFFFFFFF", panel)
    # wait for 1s
    time.sleep(1.0)
    # MIPI PCS
    self.set_register("0x02007228", "0x000C33FF", panel)
    self.set_register("0x020072C0", "0x00000001", panel)
    self.set_register("0x020072C0", "0x00000000", panel)
    # MIPI CTRL
    self.set_register("0x0200600C", "0x00000001", panel)
    self.set_register("0x020060A8", "0x00002263", panel)
    self.set_register("0x02006014", "0x00000200", panel)
    self.set_register("0x02006200", "0x00000001", panel)
    # Set MIPI image data mode
    # 0x00000000 video mode (JBD control board uses video mode)
    # 0000000001 command mode
    self.set_register("0x02006230", "0x00000000", panel)
    self.set_register("0x02006700", "0x001FFFFF", panel)
    self.set_register("0x02006704", "0x003FFFFF", panel)
    self.set_register("0x02006708", "0x00181BFF", panel)
    self.set_register("0x02006160", "0x0001FFFF", panel)
    # Enable MIPI Detection function
    self.set_register("0x02006788", "0x0200001F", panel)
    self.set_register("0x02006000", "0x00020002", panel)
    self.set_register("0x01000090", "0x00600F86", panel)
    self.set_register("0x02006000", "0x00000002", panel)
    # XDP: resolution, refresh , pixel current, algorithm setting
    # Can't set these as set_register blasts to all panels,
    # but need to set R,G,B to different values
    # MIPI signal select, b9: 0-video mode; 1-cmd mode; b14~15: 0-R, 1-G, 2-B
    if panel == JbdPanel.all or panel == JbdPanel.red:
      self.set_register("0x03040000", "0x00000103", JbdPanel.red) #R
    if panel == JbdPanel.all or panel == JbdPanel.green:
      self.set_register("0x03040000", "0x00004103", JbdPanel.green) #G
    if panel == JbdPanel.all or panel == JbdPanel.blue:
      self.set_register("0x03040000", "0x00008103", JbdPanel.blue) #B
    # set 640x480 (0x01DF = 639, 0x027F = 439)
    self.set_register("0x030000D0", "0x01DF027f", panel)
    self.set_register("0x03000080", "0x00000100", panel)
    self.set_register("0x03030300", "0x00000000", panel)
    self.set_register("0x03041004", "0x00000001", panel)
    # 54M(60Hz)
    self.set_register("0x0100003C", "0x2244003F", panel)
    self.set_register("0x0302AE90", "0x0001005A", panel)
    self.set_register("0x03030100", "0x80000011", panel)
    # Random scan off
    self.set_register("0x0303031C", "0x00000000", panel)
    self.set_register("0x03030320", "0x00000000", panel)
    self.set_register("0x03030324", "0x00000000", panel)
    self.set_register("0x03030328", "0x00000000", panel)
    self.set_register("0x0303032C", "0x00000000", panel)
    self.set_register("0x03030100", "0x80000011", panel)
    # CRG：DLL output clock
    self.set_register("0x010000AC", "0x00080005", panel)
    self.set_register("0x010000AC", "0x00080001", panel)
    self.set_register("0x010000E4", "0x00080005", panel)
    self.set_register("0x010000E4", "0x00080001", panel)
    self.set_register("0x010000EC", "0x00080005", panel)
    self.set_register("0x010000EC", "0x00080001", panel)
    self.set_register("0x010000AC", "0x00080003", panel)
    self.set_register("0x010000E4", "0x00080003", panel)
    self.set_register("0x010000EC", "0x00080003", panel)
    self.set_register("0x01000002", "0x00000000", panel)
    self.set_register("0x0100002c", "0x00000000", panel)
    # CORE CTRL：VDD, MVDD, OSCVDD selection
    self.set_register("0x02000040", "0x0020C208", panel)
    self.set_register("0x02000044", "0x0000448A", panel)
    self.set_register("0x02000048", "0x00002088", panel)
    # Auto initialization check enable
    # self.set_register("0x02000084", "0x00000002", panel)
    # CMD TOP：DCS&MCS unlock
    self.set_register("0x02001020", "0x5a5a5a5a", panel)
    # LTC function uses data stored in flash, not expected for use in RB4
    # LTC on   Brightnesss compensation with temperature variations
    # self.set_register("0x0302AE68", "0x00000001", panel)
    # self.set_register("0x02003004", "0x00B3635B", panel)
    # self.set_register("0x02003004", "0x05B1DBA5", panel)
    # LTC off
    self.set_register("0x0302AE68", "0x00000000", panel)
    # PMC
    # Set the default value.  Required before customize the value
    self.set_register("0x02003004", "0x0003635B", panel)
    # Set the min and max temperature value. Bit [19:10]: min T, bit[9:0]:max.
    # When temperature is out of the range, the errorflag will be triggered.
    self.set_register("0x02003004", "0x05B1DBA5", panel)
    # Error interruption configuration
    self.set_register("0x02000004", "0x00000202", panel)
    # FMC transfer overtime interruption (not using flash)
    # self.set_register("0x0200501C", "0x00006020", panel)
    # BCMG, Demura parameters check error interruption
    # self.set_register("0x03000090", "0xC0000000", panel)
    # MIPI Video Config, Bit 12 and 16 are used for interruption configurations.
    # They can only be configured once.
    # Red Set
    self.set_register("0x03040000", "0x00011103", JbdPanel.red)
    # Green set
    self.set_register("0x03040000", "0x00015103", JbdPanel.green)
    # Blue set
    self.set_register("0x03040000", "0x00019103", JbdPanel.blue)
    # ESD
    self.set_register("0x02000004", "0x00000200", panel)
    # Error flag pin will not be routed on RB4 flex
    # Error flag IO multiplexing setting
    self.set_register("0x01001038", "0x00000001", panel)
    # Flip/Mirror/Offset functions, set to default values used in JBD FW
    if panel == JbdPanel.all or panel == JbdPanel.red:
      self.set_register("0x03030004", "0x00000000", JbdPanel.red)
      self.set_register("0x03030008", "0x00008081", JbdPanel.red)
      self.set_register("0x0300015C", "0x20080008", JbdPanel.red)
    if panel == JbdPanel.all or panel == JbdPanel.green:
      self.set_register("0x03030004", "0x00000001", JbdPanel.green)
      self.set_register("0x03030008", "0x00008081", JbdPanel.green)
      self.set_register("0x0300015C", "0x60080008", JbdPanel.green)
    if panel == JbdPanel.all or panel == JbdPanel.blue:
      self.set_register("0x03030004", "0x00000000", JbdPanel.blue)
      self.set_register("0x03030008", "0x00008081", JbdPanel.blue)
      self.set_register("0x0300015C", "0x20080008", JbdPanel.blue)

  def get_mirror_offset_pass_thru(self, panel: JbdPanel):
    """Get the SRAM and VO L-R & U-D mirror (flip) & X & Y offset configuration.

    Uses pass_thru (register reads instead of control board built in methods).

    Args:
      panel:
        JbdPanel, to set the panel red, green, blue
        JbdPanel.all is not currently supported.

    Returns:
      JbdMirrorOffsetConfig object containing raw register values and
      extracted offset and mirror enable parameters.

    Raises:
      NotImplementedError: if panel == JbdPanel.all
    """
    if panel == JbdPanel.all:
      raise NotImplementedError("Only one panel at a time implemented")
    cfg = JbdMirrorOffsetConfig()
    cfg.sram_mirror_flip = int(self.get_register(panel, "0x03030004"), base=16)
    cfg.sram_offset = int(self.get_register(panel, "0x03030008"), base=16)
    cfg.vo = int(self.get_register(panel, "0x0300015C"), base=16)
    cfg.recalculate()
    return cfg

  def set_mirror_offset_pass_thru(self, panel: JbdPanel,
                                  x_offset=None, y_offset=None,
                                  lr_mirror=None, ud_mirror=None):
    """Set the SRAM and VO L-R & U-D mirror (flip) & X & Y offset configuration.

    Uses pass_thru (register reads instead of control board built in methods).

    Args:
      panel:
        JbdPanel, to set the panel red, green, blue
        JbdPanel.all is not currently supported.
      x_offset:
        int, the amount in pixels to offset the image in x (horizontal)
      y_offset:
        int, the amount in pixels to offset the image in y (vertical)
      lr_mirror:
        bool, enable/disable the left-right mirror function
      ud_mirror:
        bool, enable/disable the up-down mirror (flip) function

    Returns:
      None

    Raises:
      NotImplementedError: if panel == JbdPanel.all
    """
    if panel == JbdPanel.all:
      raise NotImplementedError("Only one panel at a time implemented")
    old = self.get_mirror_offset_pass_thru(panel)
    if x_offset is None:
      x_offset = old.sram_offset_x
    if y_offset is None:
      y_offset = old.sram_offset_y
    if lr_mirror is None:
      lr_mirror = old.sram_lr_mirror
    if ud_mirror is None:
      ud_mirror = old.sram_ud_mirror
    sram_mirror_flip = int(ud_mirror) << 1 | int(lr_mirror)
    #print(hex(sram_mirror_flip))
    self.set_register("0x03030004", f"0x{(sram_mirror_flip):08x}", panel)
    if x_offset > 24: x_offset = 24
    elif x_offset < 0: x_offset = 0
    if y_offset > 24: y_offset = 24
    elif y_offset < 0: x_offset = 0
    offset_en = int(x_offset > 0 or y_offset > 0)
    sram_offset = (y_offset << 12 | x_offset << 4 | offset_en)
    #print(hex(sram_offset))
    self.set_register("0x03030008", f"0x{(sram_offset):08x}", panel)
    vo = (int(ud_mirror) << 31 | int(lr_mirror) << 30 | offset_en << 29 |
          y_offset << 16 | x_offset)
    #print(hex(vo))
    self.set_register("0x0300015C", f"0x{(vo):08x}", panel)

  def control_panel_reset(self):
    """Wrapper to set the reset pin high then low.

    Args:
      None

    Returns:
      None

    Raises:
      None
    """

    self.set_reset_pin(1)
    time.sleep(0.2)  # it takes a while to reset
    self.set_reset_pin(0)
    self.logger.info("Control Board reset started, wait %s sec for board to "
                     "come back up.", RESET_DELAY)
    time.sleep(RESET_DELAY)

  def initialize_panel(self, resolution: JbdPaneResolutionIdx = None):
    """Initialize the panel.

    Uses the built in method in the control board FW and sets the resolution.

    Args:
      resolution:
        JbdPaneResolutionIdx, either JbdPaneResolutionIdx.res_640x480
        or JbdPaneResolutionIdx.res_660x504

    Returns:
      bytearray containing response message from the control board.

    Raises:
      None
    """
    if resolution:
      self.set_panel_resolution(resolution)
    self._initialize_panel()

  def _initialize_panel(self):
    """Call the built-in panel init from the ctrl brd.

    Args:
      None

    Returns:
      None

    Raises:
      None
    """
    self.logger.debug("Initialize the panel")
    ctrl_msg = JbdControlMsg.initialize_panel.value
    return self.query(self._build_msg(ctrl_msg))

  def load_gamma_tables(self):
    """Load the gamma tables (from flash memory).

    Uses the built in method in the control board FW.

    Args:
      None

    Returns:
      bytearray containing response message from the control board.

    Raises:
      None.
    """

    self.logger.debug("Load gamma tables")
    ctrl_msg = JbdControlMsg.load_gamma_data.value
    return self.query(self._build_msg(ctrl_msg))

  def set_panel_resolution(self, resolution: JbdPaneResolutionIdx,
                           panel: JbdPanel = JbdPanel.all):
    """sets the panel resolution
      (using the built in method in the control board FW)

    Args:
      resolution: JbdPaneResolutionIdx, either JbdPaneResolutionIdx.res_640x480
        or JbdPaneResolutionIdx.res_660x504
      panel: JbdPanel, any color or all

    Returns:
      None

    Raises:
      bytearray containing response message from the control board.
    """

    ctrl_msg = JbdControlMsg.set_display_resolution.value
    msg = ctrl_msg + panel.value + resolution.value
    self.logger.info("Set panel resolution to %sx%s",
                     _PANEL_RESOLUTIONS[resolution][0],
                     _PANEL_RESOLUTIONS[resolution][1])
    self.resolution_idx = resolution
    self._resolution = _PANEL_RESOLUTIONS[resolution]
    return self.query(self._build_msg(msg))

  def set_display_enable(self, enable: bool, panel: JbdPanel = JbdPanel.all):
    """Sets the panel enable state.

    Uses the built in method in the control board FW.

    Args:
      enable:
        bool, True = enable panel, False = disable panel
      panel:
        JbdPanel, any color or all

    Returns:
      bytearray containing response message from the control board.

    Raises:
      None
    """

    ctrl_msg = JbdControlMsg.display_enable.value
    msg = ctrl_msg + panel.value + bytearray([int(enable)])
    self.logger.info("Set %s display enable = %s", panel, enable)
    return self.query(self._build_msg(msg))

  def deep_power_down(self):
    """Put the panel into Deep Power Down State.

    Args:
      None

    Returns:
      None

    Raises:
      None
    """

    # Switch off AA current source
    register_address = [0x02, 0x00, 0x00, 0x30]
    register_data = [0x00, 0x00, 0x00, 0x00]
    self.set_register(register_address, register_data)
    # Switch off AA PWM Clock
    register_address = [0x03, 0x03, 0x03, 0x00]
    register_data = [0x00, 0x00, 0xFF, 0x11]
    self.set_register(register_address, register_data)
    # Switch off MIPI LDO
    register_address = [0x02, 0x00, 0x00, 0x44]
    register_data = [0x00, 0x04, 0x00, 0x00]
    self.set_register(register_address, register_data)
    # Switch off OSC LDO
    register_address = [0x02, 0x00, 0x00, 0x48]
    register_data = [0x00, 0x04, 0x00, 0x00]
    self.set_register(register_address, register_data)

  def wake(self):
    """Wake all connected panels out of Deep Power Down State.

    Args:
      None

    Returns:
      None

    Raises:
      None
    """
    self.set_reset_pin(0)
    self.initialize_panel()

  def i2c_interface_enable(self):
    """Enable the control board I2C interface.

    Args:
      None

    Returns:
      None

    Raises:
      None
    """
    ctrl_msg = JbdControlMsg.i2c_interface_enable.value
    self.logger.debug("i2c interface enable")
    return self.query(self._build_msg(ctrl_msg))

  def get_control_board_firmware(self):
    """Get the firmware version from the ctrl brd.

    Uses the built in method in the JBD control board FW.

    Args:
      None

    Returns:
      bytearray, from self.query(), containing the control board response.
      bytes[1:9] are the characters that represent the control board version

    Raises:
      None
    """
    ctrl_msg = JbdControlMsg.get_firmware.value
    return self.query(self._build_msg(ctrl_msg))

  def check_control_board_fw_version(self):
    """Get the firmware version from the ctrl brd.

    Uses the built in method in the JBD control board FW.

    Args:
      None

    Returns:
      cb_ver_int:
        int, value representing the enumeration of the FW version, -1: unknown
        version
      cb_ver_str:
        str, decoded string from the control board response to get FW version

    Raises:
      None
    """
    cb_ver_str = self.get_control_board_firmware()[1:9].decode(encoding='UTF-8',
                                                               errors='ignore')
    if cb_ver_str in CB_FW_VERSIONS:
      cb_ver_int = CB_FW_VERSIONS[cb_ver_str]
    else:
      cb_ver_int = -1

    latest_cb_ver_int = max(CB_FW_VERSIONS.values())
    latest_cb_ver_str = ([k for k, v in CB_FW_VERSIONS.items() if v ==
                         latest_cb_ver_int][0])
    if cb_ver_int == -1:
      self.logger.warning("Control Board FW version is %s.  This version is "
                          "unknown.  The latest known version is %s",
                          cb_ver_str, latest_cb_ver_str)
    elif cb_ver_int < latest_cb_ver_int :
      self.logger.warning("Control Board FW version is %s.  The latest known "
                          "version is %s", cb_ver_str, latest_cb_ver_str)
    else:
      self.logger.info("Control Board FW version is %s.  This is the latest "
                       "known version.", cb_ver_str)

    return int(cb_ver_int), cb_ver_str


  def _read_temp_sensor(self, panel: JbdPanel):
    """gets the ADC value from the temp sensor.

    Uses the built in control board FW method.

    Args:
      panel:
        JbdPanel, any color (all not currently supported)

    Returns:
      bytearray containing the ADC value read

    Raises:
      NotImplementedError: if panel == JbdPanel.all
    """
    if panel == JbdPanel.all:
      raise NotImplementedError("Only one panel at a time implemented")
    ctrl_msg = JbdControlMsg.read_temp_sensor.value + panel.value
    resp = self._parse_data_flow(self.query(self._build_msg(ctrl_msg)))
    return resp[1:]

  def get_panel_temperature(self, panel: JbdPanel):
    """Reads the temp sensor and returns a value in degrees Celsius.

    Args:
      panel:
        JbdPanel, any color (all not currently supported)

    Returns:
      float, temperature in degC

    Raises:
      NotImplementedError: if panel == JbdPanel.all
    """
    resp = self._read_temp_sensor(panel)
    return round(self._calc_temp_from_value(self._bytes_to_int(resp)), 4)

  def _read_eFUSE(self, panel: JbdPanel, read_byte: int):
    """Read the eFUSED bits.
       (reading efuse bits requires a differen sequence than reading a
       register)

    Args:
      panel:
        JbdPanel, any color (all not currently supported)

    Returns:
      byte read

    Raises:
      NotImplementedError: if panel == JbdPanel.all
    """
    if panel == JbdPanel.all:
      raise NotImplementedError("Only one panel at a time implemented")
    #panel = JbdPanel.all
    # select byte to read out from eFUSE
    self.set_register([0x02, 0x00, 0x90, 0x0C],
                      [0x00, 0x00, 0x00, read_byte],
                      panel)

    # eFUSE enable
    self.set_register([0x02, 0x00, 0x90, 0x08],
                      [0x00, 0x00, 0x00, 0x01],
                      panel)
    time.sleep(0.04)
    #return self.get_register(panel, [0x02, 0x00, 0x90, 0x14])[0]
    read_ba = self.get_register(panel, [0x02, 0x00, 0x90, 0x14])
    #print(read_ba)
    if len(read_ba) > 0:
      return read_ba[-1]
    else:
      return None

  def _read_creg(self, panel: JbdPanel):
    """Read the creg (current setting register).

    Uses direct register read and returns the raw register data.

    Args:
      panel: JbdPanel, any color

    Returns:
      bytearray (4bytes to 12bytes) read

    Raises:
      None
    """
    return self.get_register(panel, [0x03, 0x02, 0xAE, 0x90])

  def get_current(self, panel: JbdPanel):
    """Read the current setting (0x00 to 0xFF).

    Uses direct register read and returens a formatted output.

    Args:
      panel:
        JbdPanel, any color or all

    Returns:
      str or list of 3 strings representing the current register value as a
        single byte in hex format

    Raises:
      None
    """
    read_data = self.get_register(panel, "0x0302AE90")
    if panel == JbdPanel.all:
      value_list = []
      for c in ["red", "green", "blue"]:
        str = read_data.find(c) + len(c) + 2
        end = str + 11
        val_int = hex(int(read_data[str:end], 16) & 0xFF)
        value_list.append(val_int)
      return value_list
    else:
      return hex(int(read_data, 16) & 0xFF)

  def read_flash_id(self, panel: JbdPanel):
    """Read flash id using register write and read commands.

    Args:
      panel:
        JbdPanel, any color ("all" not implemented)

    Returns:
      str, containing the 4 byte read back data value
      (or 3x 4 bytes if JbdPanel is all)

    Raises:
      None
    """
    self.set_register("0x02005020", "0x000000FF", panel)
    self.set_register("0x02005024", "0x0000009F", panel)
    self.set_register("0x02005030", "0x00000030", panel)
    self.set_register("0x02005038", "0x00000003", panel)
    self.set_register("0x0200503C", "0x00000085", panel)
    return(self.get_register(panel, "0x04000000"))

  def read_die_id(self, panel: JbdPanel):
    """Read the die ID using the JBD control board built in method.

    Args:
      panel:
        JbdPanel, any color or all

    Returns:
      byte array containing the UART response message

    Raises:
      None
    """
    ctrl_msg = JbdControlMsg.read_die_id.value + panel.value
    return self._parse_data_flow(self.query(self._build_msg(ctrl_msg)))

  def read_die_id_raw(self, panel: JbdPanel):
    """Read die id from eFUSE using register write and read commands.

    
    Args:
      panel:
        JbdPanel, any color ("all" not implemented)

    Returns:
      formatted_die_id:
        str, formatted per JBD definition (ex: A040X04C0422123103B30).

    Raises:
      NotImplementedError: If panel == JbdPanel.all
    """

    if panel == JbdPanel.all:
      raise NotImplementedError("Only one panel at a time implemented")
    die_id = []
    for i in range(115, 127+1):
      die_id.append(self._read_eFUSE(panel, i))
    #print([hex(x) for x in die_id])
    if die_id[0] is None:
      return f"No ID readback. Check {panel} panel is connected."
    else:
      for i in range(0, 12 + 1):
        #print(die_id)
        if i == 0 or i == 3 or i == 5 or i == 11:
          die_id[i] = (chr(die_id[i]))
        else:
          die_id[i] = (f"{die_id[i]:02X}")
      # print(die_id)
      formatted_die_id = "".join(die_id)
      return formatted_die_id

  def read_ic_version(self, panel: JbdPanel):
    """Read IC version directly from eFUSE.
    
    Args:
      panel: JbdPanel, any color ("all" not implemented)

    Returns:
      ic_id: str, i.e. "xx4020" or "xx4021" or "000" if unprogrammed.

    Raises:
      NotImplementedError
    """

    if panel == JbdPanel.all:
      raise NotImplementedError("Only one panel at a time implemented")
    ic_id = []
    for i in range(0x35, 0x37 + 1):
      ic_id.append(self._read_eFUSE(panel, i))
    #print([hex(x) for x in ic_id])
    if ic_id[0] is None:
      return f"No IC ID readback. Check {panel} panel is connected."
    else:
      return f"{ic_id[2]}{ic_id[1]}{ic_id[0]}"

  def read_i2c_efuse(self, panel: JbdPanel):
    """Read I2C programming directly from eFUSE.
    
    Args:
      panel: JbdPanel, any color ("all" not implemented)

    Returns:
      i2c_en: bool, True if efuse bit to enable I2C is = 1
      i2c_addr: str, selected I2C address (str hex value)

    Raises:
      NotImplementedError
    """

    if panel == JbdPanel.all:
      raise NotImplementedError("Only one panel at a time implemented")
    ic_id = []
    for i in range(0x15, 0x15 + 1):
      ic_id.append(self._read_eFUSE(panel, i))
    #print([hex(x) for x in ic_id])
    if ic_id[0] is None:
      return f"No I2C efuse readback. Check {panel} panel is connected."
    else:
      i2c_addr = (0x60 & int(ic_id[0])) >> 5
      i2c_en = bool((0x04 & int(ic_id[0])) >> 2)
      i2c_addr_map = {
        0: "0x58",
        1: "0x59",
        2: "0x5A",
        3: "0x5B"
      }
      return i2c_en, i2c_addr_map[i2c_addr]
      #return f"{ic_id[0]:02X}"


  def set_luminance(self, value: int, panel: JbdPanel = JbdPanel.all):
    """Set the panel luminance setting (aka DBV aka Lreg) 0-8191.

    Uses the built in method in the FW (As opposed to a raw write to register).
    Note gamma must be enabled and gamma LUT set appropriately in order for this
    function to work.

    Args:
      value:
        int, luminance value 0 to 8191.  Values out of range will be clamped to
        [0, 8191]
      panel:
        JbdPanel, any color or all

    Returns:
      bytearray containing the response on UART

    Raises:
      None
    """
    if value < 0 : value = 0
    elif value > 8191: value = 8191

    ctrl_msg = (JbdControlMsg.write_lreg.value + panel.value +
                value.to_bytes(2, "big"))
    return self.query(self._build_msg(ctrl_msg))

  def get_luminance(self, panel: JbdPanel = JbdPanel.all):
    """Read the panel luminance setting (aka DBV aka Lreg) 0-8191.

    Args:
      panel:
        JbdPanel, any color or all

    Returns:
      str or list of 3 str representing the luminance setting as a hex value

    Raises:
      None
    """
    read_data = self.get_register(panel, "0x0302AE38")
    #print(read_data)
    if panel == JbdPanel.all:
      value_list = []
      for c in ["red", "green", "blue"]:
        str = read_data.find(c) + len(c) + 3
        end = str + 10
        #print(read_data[str:end])
        if "None" not in read_data[str:str+4]:
          val_str = hex(int(read_data[str:end], 16) & int("0x00001FFF", 16))
        else:
          val_str = None
        value_list.append(val_str)
      return value_list
    else:
      return hex(int(read_data, 16) & int("0x00001FFF", 16))

  def set_lr_mirror(self, value: int, panel: JbdPanel = JbdPanel.all):
    """Set the panel's left right mirror function.  Uses the control board built
    in function.

    Uses the control board built in function.

    Args:
      value: int, 1: enable left/right mirror. 0: disable up/down mirror
      panel: JbdPanel, any color or all

    Returns:
      self.query(): bytearray

    Raises:
      None
    """
    if value != 1 and value != 0:
      raise ValueError("Mirror function expect 0 or 1 input")
    ctrl_msg = (JbdControlMsg.set_h_mirror.value + panel.value +
                bytearray([value]))
    print(f"Writing to LR mirror: {bytearray([value])}")
    return self.query(self._build_msg(ctrl_msg))

  def set_ud_mirror(self, value: int, panel: JbdPanel = JbdPanel.all):
    """Set the panel's up down mirror (flip) function.  Uses the control board
    built in function.

    Args:
      value:
        int, 1: enable up/down mirror. 0: disable up/down mirror
      panel:
        JbdPanel, any color or all

    Returns:
      self.query(), bytearray

    Raises:
      None
    """
    if value != 1 and value != 0:
      raise ValueError("Mirror function expect 0 or 1")
    ctrl_msg = (JbdControlMsg.set_v_mirror.value + panel.value +
                bytearray([value]))
    return self.query(self._build_msg(ctrl_msg))

  def set_random_scan(self, value: int, panel: JbdPanel = JbdPanel.all):
    """Set random scan on/off.  Uses the control board built in function.

    Args:
      value:
        int, 1: enable random scan. 0: disable random scan.
      panel:
        JbdPanel, any color or all

    Returns:
      self.query(), bytearray

    Raises:
      None
    """
    ctrl_msg = None
    if value >= 1: value = 1
    else: value = 0
    ctrl_msg = (JbdControlMsg.random_scan_control.value + panel.value
                + bytearray([value]))
    return self.query(self._build_msg(ctrl_msg))

  def set_dither(self, value: int, panel: JbdPanel = JbdPanel.all):
    """Set dither on/off.  Uses the control board built in function.
      Enabling dither will set the Spatial Only mode.
      

    Args:
      value:
        int. 1: enable dither. 0: disable dither.
      panel: R/G/B

    Returns:
      self.query(), bytearray

    Raises:
      None
    """

    ctrl_msg = None
    if value >= 1: value = 1
    else: value = 0
    ctrl_msg = (JbdControlMsg.dither_enable.value + panel.value
                + bytearray([value]))
    return self.query(self._build_msg(ctrl_msg))

  def set_current(self, value: int, panel: JbdPanel = JbdPanel.all):
    """Set the panel current (aka Creg) 0-255.

    Uses the built in method in the FW (As opposed to a raw write to register).

    Args:
      value:
        int, current value 0 to 255.  Values out of range will be clamped to
        [0, 255]
      panel:
        JbdPanel, any color or all

    Returns:
      bytearray containing the response on UART

    Raises:
      None
    """
    if value < 0: value = 0
    elif value > 255: value = 255
    ctrl_msg = JbdControlMsg.write_creg.value + panel.value + bytearray([value])
    return self.query(self._build_msg(ctrl_msg))

  def _write_creg(self, value: int, panel: JbdPanel = JbdPanel.all):
    """Set the panel current (aka Creg) 0-255.

    Uses a direct write to the register as opposed to built in control board
    FW method.  Note this means that in order to have the change take effect,
    the image will need to be updated (resent).  This will happen automatically
    if HDMI is enabled.

    Args:
      value:
        int, current value 0 to 255.  Values out of range will be clamped to
        [0, 255]
      panel:
        JbdPanel, any color or all

    Returns:
      None

    Raises:
      None
    """
    if value < 0: value = 0
    elif value < 255: value = 255
    #print(f"0x{(0x00010000 | value):08x}")
    self.set_register("0x0302AE90", f"0x{(0x00010000 | value):08x}", panel)

  def set_register(self, reg_address: Union[list, str],
                   reg_data: Union[list, str],
                   panel: JbdPanel = JbdPanel.all,
                   force_old_fw: bool = False):
    """Set (write) register address and data.
    Please note that Control Board FW version >= 1.14.21 is required in order to
    support per color register writes.  Earlier FW versions will fail to
    function if panel != JbdPanel.all

    Args:
      reg_address:
        list of 4 bytes, or str of 4 bytes as "0xXXXXXXXX"
      reg_data:
        list of 4 bytes, or str of 4 bytes as "0xXXXXXXXX"
      panel:
        JbdPanel, any color or all

    Returns:
      None

    Raises:
      ValueError: if panel type not red, green, blue, or all
      NotImplemented Error: if FW version < V1.14.21 and panel type is not all
    """
    #force_old_fw = True
    if panel == JbdPanel.all:
      cmd_byte = JbdControlMsg.write_register_all
    elif panel == JbdPanel.red:
      cmd_byte = JbdControlMsg.write_register_red
    elif panel == JbdPanel.green:
      cmd_byte = JbdControlMsg.write_register_green
    elif panel == JbdPanel.blue:
      cmd_byte = JbdControlMsg.write_register_blue
    else:
      self.logger.error("Invalid %s type", panel)
      raise ValueError(f"The {panel} panel type is not supported."
                       f"Expect one of: {[e.value for e in JbdPanel]}")
    if (self.cb_fw_version[0] < 5 and (panel != JbdPanel.all)):
      if force_old_fw:
        panel = JbdPanel.all
        cmd_byte = JbdControlMsg.write_register_all
        self.logger.warning("Per color register writes not supported in this FW"
                            " version, overriding to use write all panels.")
      else:
        self.logger.error("Per color register writes not supported in this FW "
                          "version.  Please up date to a newer version.")
        raise NotImplementedError("Per color register writes not supported in "
                                  "this FW version.  Please up date to a newer "
                                  "version.")

    #print(f"address={reg_address}, data={reg_data}")
    if isinstance(reg_address, str):
      reg_address_str = reg_address
      reg_address = []
      reg_address_str = reg_address_str.replace("0x", "")
      #print(reg_address_str)
      for i in range(0, 8, 2):
        #print(reg_address_str[i:i+2])
        reg_address.append(int(reg_address_str[i:i+2], 16))
    #print(reg_address)
    #print([hex(x) for x in reg_address])
    if isinstance(reg_data, str):
      reg_data_str = reg_data
      reg_data = []
      reg_data_str = reg_data_str.replace("0x", "")
      #print(reg_data_str)
      #for i in range(8,0,-2):
        #print(reg_data_str[i-2:i])
      #  reg_data.append(int(reg_data_str[i-2:i], 16))
      for i in range(0, 8, 2):
        #print(reg_data_str[i:i+2])
        reg_data.append(int(reg_data_str[i:i+2], 16))
    #print(reg_data)
    #print([hex(x) for x in reg_data])
    address_and_data = [*reg_address, *reg_data]
    #print(f"address={[hex(x) for x in reg_address]}, "
    #      f"data={[hex(x) for x in reg_data]}")
    ctrl_msg = cmd_byte.value + bytearray(address_and_data)
    #input()
    return self.query(self._build_msg(ctrl_msg))

  def get_register(self, panel: JbdPanel, reg_address: list | str, debug=False):
    """Get (read) register address and data.

    Args:
      reg_address:
        list of 4 bytes, or str of 4 bytes as "0xXXXXXXXX"
      panel:
        JbdPanel, any color or all
      debug:
        bool, False by default.  If true, will return data for all connected
        panels.  See Appendix 1 in Comm Protocol V2.2.4.  For use in
        non-standard configurations (i.e. more than one panel of same I2C
        address attached to the B2B connectors)

    Returns:
      bytearray (if input is list) or str (if input is string)

    Raises:
      None
    """
    #debug = True
    if isinstance(reg_address, str):
      reg_address_str = reg_address
      reg_address_list = []
      reg_address_str = reg_address_str.replace("0x", "")
      for i in range(0, 8, 2):
        reg_address_list.append(int(reg_address_str[i:i+2], 16))
    else:
      reg_address_list = reg_address
    ctrl_msg = (JbdControlMsg.read_register.value + panel.value +
                bytearray(reg_address_list))
    resp = self._parse_data_flow(self.query(self._build_msg(ctrl_msg)))

    reads = JbdReadResponse()
    reads_fields = fields(reads)
    #print(reads)
    #print(len(reads_fields))
    i = 0
    while i < len(resp):
      setattr(reads, reads_fields[int(resp[i])].name, resp[i+1:i+5])
      i += 5
    #print(reads)

    if debug:
      if isinstance(reg_address, str) and isinstance(resp, bytearray):
        for var in reads_fields:
          vn = var.name
          vv = getattr(reads, vn)
          if vv is not None:
            vhex = '0x' + ''.join(format(b, '02x') for b in vv)
          else:
            vhex = None
          print(f"{vn} = {vhex}, ", end="")
        print("")
    else:
      if isinstance(reg_address, str) and isinstance(resp, bytearray):
        # resp_str = "0x"
        # for val in resp[1:]:
        #   #print(val)
        #   resp_str = resp_str + f"{val:02x}"
        # if panel == JbdPanel.all:
        #   if resp[0] == 0:
        #     # response string is complete
        #     pass
        #   elif resp[0] == int(JbdPanel.all.value, 16):
        #     resp_str
        #   if len(resp_str) < 19:
        #     resp_str += "00000000"
        #   if len(resp_str) < 27:
        #     resp_str += "00000000"
        #   print(resp_str)
        #   return (f"red = 0x{resp_str[2:10]}, green = 0x{resp_str[10:18]}, "
        #           f"blue = 0x{resp_str[18:26]}")
        # else:
        #   return resp_str

        # prioritize reporting what is connected to the green B2B
        # (we make an arbitrary assumption that most people will be working
        # with a projector and red & blue will connect to control board via
        # the green FPC)
        if reads.g_b2b_r_0x59 is not None:
          red_resp_str = reads.g_b2b_r_0x59
        elif reads.r_b2b_r_0x59 is not None:
          red_resp_str = reads.r_b2b_r_0x59
        elif reads.b_b2b_r_0x59 is not None:
          red_resp_str = reads.b_b2b_r_0x59
        else:
          red_resp_str = None
        if red_resp_str is not None:
          vv = red_resp_str
          red_resp_str = '0x' + ''.join(format(b, '02x') for b in vv)

        if reads.g_b2b_g_0x5A is not None:
          green_resp_str = reads.g_b2b_g_0x5A
        elif reads.r_b2b_g_0x5A is not None:
          green_resp_str = reads.r_b2b_g_0x5A
        elif reads.b_b2b_g_0x5A is not None:
          green_resp_str = reads.b_b2b_g_0x5A
        else:
          green_resp_str = None
        if green_resp_str is not None:
          vv = green_resp_str
          green_resp_str = '0x' + ''.join(format(b, '02x') for b in vv)

        if reads.g_b2b_b_0x5B is not None:
          blue_resp_str = reads.g_b2b_b_0x5B
        elif reads.r_b2b_b_0x5B is not None:
          blue_resp_str = reads.r_b2b_b_0x5B
        elif reads.b_b2b_b_0x5B is not None:
          blue_resp_str = reads.b_b2b_b_0x5B
        else:
          blue_resp_str = None
        if blue_resp_str is not None:
          vv = blue_resp_str
          blue_resp_str = '0x' + ''.join(format(b, '02x') for b in vv)

        if panel == JbdPanel.all:
          return (f"red = {red_resp_str}, green = {green_resp_str}, "
                   f"blue = {blue_resp_str}")
        elif panel == JbdPanel.red:
          return red_resp_str
        elif panel == JbdPanel.green:
          return green_resp_str
        elif panel == JbdPanel.blue:
          return blue_resp_str
        else:
          return None
      else:
        return resp[1:]

  def disable_lreg_and_gamma(self, panel: JbdPanel = JbdPanel.all):
    """Disable Global brighness adjustment and Disable Gamma function.

    Uses direct register writes.

    Args:
      panel:
        JbdPanel, any color or all

    Returns:
      None

    Raises:
      None
    """
    register_address = [0x03, 0x02, 0xAE, 0x00]
    register_data = [0x00, 0x00, 0x03, 0x00]
    self.set_register(register_address, register_data, panel)

  def enable_lreg_and_gamma(self, panel: JbdPanel = JbdPanel.all):
    """Enable Global brighness adjustment and Disable Gamma function.

    Uses direct register writes.

    Args:
      panel:
        JbdPanel, any color or all

    Returns:
      None

    Raises:
      None
    """
    #Enable Global brightness adjustment and Enable Gamma function
    register_address = [0x03, 0x02, 0xAE, 0x00]
    register_data = [0x00, 0x00, 0x03, 0x80]
    self.set_register(register_address, register_data, panel)
    #disable internal dimming function
    register_address = [0x03, 0x02, 0xAE, 0x04]
    register_data = [0x01, 0x04, 0x10, 0x1E]
    self.set_register(register_address, register_data, panel)

  def gammma_on_off(self, enable: bool, setting:GammaSetting,
                    panel: JbdPanel = JbdPanel.all):
    """Enable / disable Gamma using the built in Control Board method.

    Updates all panels connected to the control board.  No support for
    individual panels at this time.  Note that a luminance setting (DBV) and a
    frame buffer update may be required in order for the gamma change to take
    effect.

    Args:
      enable:
        bool, True = turn gamma on, False = turn gamma off
      setting:
        GammaSetting,
        GammaSetting.gs_1_0_lreg_1_0 => Greyscale gamma = 1.0, Lreg gamma = 1.0
        GammaSetting.gs_2_2_lreg_1_0 => Greyscale gamma = 2.2, Lreg gamma = 1.0
        GammaSetting.gs_2_2_lreg_2_2 => Greyscale gamma = 2.2, Lreg gamma = 2.2
      panel:


    Returns:
      bytearray, from self.query(), the response on the UART

    Raises:
      NotImplementedError:
        if panel != JbdPanel.all or FW version < V1.14.G0

    """
    if panel != JbdPanel.all:
      raise NotImplementedError("JBD has not implemented Gamma setting/on/off "
                                "function for individual panels.")
    if self.cb_fw_version[0] < 6:
      self.logger.error("Gamma setting/on/off not supported in this FW "
                        "version.  Please up date to a newer version.")
      raise NotImplementedError("Gamma setting/on/off not supported in "
                                "this FW version.  Please up date to a newer "
                                "version.")
    #print(enable)
    #print(setting)
    ctrl_msg = (JbdControlMsg.gamma_on_off.value + bytearray([int(enable)]) +
                setting)
    #print(ctrl_msg)
    return self.query(self._build_msg(ctrl_msg))

  def clear_screen(self, panel: JbdPanel = JbdPanel.all):
    """Clear screen for all connected panels.

    Uses the built in method in the JBD control board FW.

    Args:
      panel:
        JbdPanel, any color or all

    Returns:
      self.query: bytearray containg the UART response

    Raises:
      None
    """
    if panel != JbdPanel.all:
      raise NotImplementedError("JBD has not implemented clear screen function"
                                "for individual panels.")
    ctrl_msg = JbdControlMsg.clear_display.value
    return self.query(self._build_msg(ctrl_msg))

  def set_gamma_tables(self, setting:str = None,
                       panel: JbdPanel = JbdPanel.all):
    """Set the gamma tables.

    Args:
      setting:
        str, "disable" will disable lreg and gamma. A path to a csv file
        will load the gamma lut address/data pairs listed in the file and
        enable lreg and gamma.
      panel:
        JbdPanel, any color or all

    Returns:
      None

    Raises:
      None
    """
    if setting is None or setting == "disable":
      self.disable_lreg_and_gamma(panel)
    else:
      self.write_gamma_tables(setting, panel)
      self.enable_lreg_and_gamma(panel)

  def write_gamma_tables(self, gamma_csv_path=None,
                         panel: JbdPanel = JbdPanel.all):
    """Write the gamma tables defined in a CSV file.

    Please note this function can be very slow as the CSV file for the gamma LUT
    often contains over 1000 register address/data pairs.

    Args:
      gamma_csv_path:
        str, a path to a csv file defining the gamma LUT as address/data pairs
        listed in the file.
      panel:
        JbdPanel, any color or all.

    Returns:
      None

    Raises:
      None
    """
    if gamma_csv_path is None:
      current_file_directory = os.path.dirname(os.path.abspath(__file__))
      gamma_file = "gamma1.0_2.csv"
      gamma_csv_path = os.path.join(current_file_directory, gamma_file)
    gamma_tables = np.genfromtxt(gamma_csv_path, delimiter=",",
                                 dtype=str, skip_header=1, comments="#")
    for row in gamma_tables:
      self.set_register(row[0], row[1], panel)

  def read_gamma_tables(self, panel: JbdPanel, gamma_csv_path=None):
    """Read the gamma tables to a file

    Args:
      panel:
        JbdPanel = any color (all not supported)
      gamma_csv_path:
        str = path to file to write gamma table to.  Defaults to
        "gamma_read.csv" in the current file directory.

    Returns:
      None

    Raises:
      NotImplementedError
    """
    # if panel == JbdPanel.all:
    #   raise NotImplementedError("Read Gamma Tables has not been implemented, "
    #                             "for the 'all' panels option")
    # current_file_directory = os.path.dirname(os.path.abspath(__file__))
    # gamma_addr_file = "gamma_tables_addresses.csv"
    # gamma_addr_path = os.path.join(current_file_directory, gamma_addr_file)
    # gamma_addr = np.genfromtxt(gamma_addr_path, delimiter=",", dtype=str,
    #                            skip_header=1)
    # # gamma_val = np.empty(np.shape(gamma_addr), dtype=str)
    # # print(np.shape(gamma_val))
    # gamma_val = []
    # for i, row in enumerate(gamma_addr):
    #   row_0x_strip = row.replace("0x", "")
    #   addr_bytearray = bytearray.fromhex(row_0x_strip)
    #   data_bytearray = self.get_register(panel, addr_bytearray)
    #   # print(data_bytearray)
    #   data_str = bytes(data_bytearray).hex()
    #   # print(data_str)
    #   # gamma_val[i] = '0x' + data_str
    #   gamma_val.append("0x" + data_str)
    # #print(gamma_val)
    # if gamma_csv_path is None:
    #   gamma_read_path = os.path.join(current_file_directory, "gamma_read.csv")
    # else:
    #   gamma_read_path = gamma_csv_path
    # np.savetxt(gamma_read_path, np.column_stack((gamma_addr, gamma_val)),
    #            delimiter=",", fmt="%s", header="Address, Value")
    current_file_directory = os.path.dirname(os.path.abspath(__file__))
    gamma_addr_file = "gamma_tables_addresses.csv"
    gamma_addr_path = os.path.join(current_file_directory, gamma_addr_file)
    if gamma_csv_path is None:
      gamma_read_path = os.path.join(current_file_directory, "gamma_read.csv")
    else:
      gamma_read_path = gamma_csv_path
    self.read_registers(panel, gamma_addr_path, gamma_read_path)

  def read_registers(self, panel: JbdPanel, address_path, output_path):
    """Read a list of register values.

    Read a list of registers addresses from a CSV file and write a list of
    register/data pairs to a CSV file.

    Args:
      panel:
        JbdPanel, any color (all not supported).
      address_path:
        str, path to csv file listing the register addresses to read.
      output_path:
        str, path to csv file name to save the list of read register/data pairs
        to.

    Returns:
      None

    Raises:
      NotImplementedError
    """
    if panel == JbdPanel.all:
      raise NotImplementedError("Read Registers has not been implemented, "
                                "for the 'all' panels option")
    addr = np.genfromtxt(address_path, delimiter=",", dtype=str,
                               skip_header=1)
    read_val = []
    for i, row in enumerate(addr):
      row_0x_strip = row.replace("0x", "")
      addr_bytearray = bytearray.fromhex(row_0x_strip)
      data_bytearray = self.get_register(panel, addr_bytearray)
      # print(data_bytearray)
      data_str = bytes(data_bytearray).hex()
      # print(data_str)
      # read_val[i] = '0x' + data_str
      read_val.append("0x" + data_str)
    print(read_val)
    np.savetxt(output_path, np.column_stack((addr, read_val)),
               delimiter=",", fmt="%s", header="Address, Value")

  def draw_rectangle(self, start_col: int, start_row: int,
                     end_col: int, end_row: int, gray: int = 255):
    """Draw an EMPTY (wire frame) box with defined grayscale level.

    Box is defined by corner coordinates:
    top left corner (x, y) = (start_col, start_row)
    bottom left corner (x, y)  = (end_col, end_row)

    Args:
      start_col:
        int, starting column (x) pixel position
      start_row:
        int, starting row (y) pixel position
      end_col:
        int, ending column (x) pixel position
      end_row:
        int, ending row (y) pixel position

    Returns:
      None

    Raises:
      None
    """
    # convert the values to ints in case they're floats
    start_col = int(start_col)
    start_row = int(start_row)
    end_col = int(end_col)
    end_row = int(end_row)
    # put values into 3-bit segments
    v1 = (start_row >> 4) & 0x00ff
    v2 = ((start_row & 0x000f) << 4) | ((start_col >> 8) & 0x000f)
    v3 = start_col & 0x00ff
    v4 = (end_row >> 4) & 0x00ff
    v5 = ((end_row & 0x000f) << 4) | ((end_col >> 8) & 0x000f)
    v6 = end_col & 0x00ff
    ctrl_msg = JbdControlMsg.draw_rectangle.value + bytearray(
      [v1, v2, v3, v4, v5, v6, gray])
    return self.query(self._build_msg(ctrl_msg)).decode(
      errors="backslashreplace")

  def write_monochrome_image(self, image_path: str, grayscale_clip: int = None):
    """Load and write monochrome image from local disk to the panel.
      Each color channel will receive the same greyscale data.

    Each color channel will receive the same greyscale data.

    Args:
      image_path:
        str, path to an image file
      grayscale_clip:
        int, clip the pixels to this value 0-255 (clip not scale)

    Returns:
      None

    Raises:
      FileNotFoundError: if image_path not founde
      ValueError: if image shape does not match expected resolution
    """

    if not os.path.exists(image_path):
      raise FileNotFoundError(image_path)
    img = cv2.imread(image_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if grayscale_clip is not None:
      if grayscale_clip > 255 : grayscale_clip = 255
      img = np.clip(img, 0, grayscale_clip)
    image_msg = bytearray(img.flatten())
    rows = int(0).to_bytes(2, "big")  # NOTE These are not used per the API
    cols = int(0).to_bytes(2, "big")
    self.logger.debug("image shape: %s", img.shape)
    if self._resolution:
      expected_image_mgs_len = self._resolution[0] * self._resolution[1]
      if expected_image_mgs_len != len(image_msg):
        raise ValueError(f"The provided image shape is not supported. "
                         f"Expect: {self._resolution}")
    else:
      self.logger.warning("Writing image with unknown resolution settings.")
    flow_len = len(image_msg).to_bytes(4, "big")
    msg = (JbdControlMsg.display_mono_image.value + rows + cols + flow_len +
           CMD_SUFFIX)
    chk = self._calc_xor_checksum(image_msg)
    msg = msg + image_msg + chk
    self.query(msg, response_delay=2)

  def set_panel_refresh_rate(self, refresh_rate: RefreshRate,
                             panel: JbdPanel = JbdPanel.all):
    """Set the panel refresh rate.

    Args:
      refresh_rate:
        RefreshRate = 30, 37.5, 50, 60, or 75Hz
      panel:
        JbdPanel = any color or all

    Returns:
      None

    Raises:
      FileNotFoundError
      ValueError

    """
    vdp_tcon_config_reg = [0x01, 0x00, 0x00, 0x3C]
    self.set_register(vdp_tcon_config_reg, refresh_rate.value, panel=panel)

  def set_mipi_refresh_rate(self, mipi_refresh: MipiRefreshConfig,
                            panel: JbdPanel = JbdPanel.all):
    """Set the MIPI refresh rate or the Panel Rx.

    Note the JBD control board is set to 60Hz MIPI DSI Video mode and this is
    not adjustable.

    Args:
      mipi_refresh:
        MipiRefreshConfig = set the expected input MIPI DSI refresh rate
      panel:
        JbdPanel = any color or all

    Returns:
      None

    Raises:
      None

    """
    mipi_config_reg = [0x01, 0x00, 0x00, 0x90]
    #if mipi_refresh.value == MipiRefreshConfig.mipi_refresh_75:
    # fix to "75Hz" setting as JBD has updated their recommendation to use this
    # value for 60Hz input and higher
    self.set_register(mipi_config_reg,
                        MipiRefreshConfig.mipi_refresh_75.value)
    #else:
    #  self.set_register(mipi_config_reg,
    #                    MipiRefreshConfig.mipi_refresh_other.value)

  def write_color_image(self, image_path, grayscale_clip=None):
    """Load and write color image from local disk to the panel.

    This method is not implemented in JBD FW. Use write_monochrome_image
    or write_color_image_with_rle instead.

    Args:
      image_path:
        str, path to file
      grayscale_clip:
        int, clip max image value to this level 0-255

    Returns:
      None

    Raises:
      NotImplementedError

    """
    raise NotImplementedError("This method is not implemented"
                              " in JBD FW. Use write_monochrome_image"
                              " or write_color_image_with_rle instead.")
    # code below will fail to draw an image because the JBD control board
    # fails to respond to the JbdControlMsg.display_color_image.value (0x35)
    # command
    if not os.path.exists(image_path):
      raise FileNotFoundError(image_path)
    img = cv2.imread(image_path)
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # according to Comm protocol 4020 V2.2.1 section 2.1, the data format is BGR
    # not RGB
    
    if grayscale_clip is not None:
      img = np.clip(img, 0, grayscale_clip)
    image_msg = bytearray(img.flatten())
    self.logger.debug(f'image_msg number of bytes = {len(image_msg)}')
    rows = int(0).to_bytes(2, "big")  # NOTE These are not used per the API
    cols = int(0).to_bytes(2, "big")
    self.logger.debug("image shape: %s", img.shape)
    if self._resolution:
      expected_image_mgs_len = self._resolution[0] * self._resolution[1] * 3
      if expected_image_mgs_len != len(image_msg):
        raise ValueError(f'The provided image shape is not supported. '
                         f'Expect: {self._resolution}')
    else:
      self.logger.warning("Writing image with unknown resolution settings.")
    flow_len = len(image_msg).to_bytes(4, "big")
    self.logger.debug(f'flow len = {flow_len}')
    msg = (JbdControlMsg.display_color_image.value + rows + cols + flow_len +
           CMD_SUFFIX)
    chk = self._calc_xor_checksum(image_msg)
    msg = msg + image_msg + chk
    self.logger.debug(f'msg number of bytes = {len(msg)}')
    self.query(msg, response_delay=2)

  def write_color_image_with_rle(self, image_path: str,
                                 grayscale_clip: int = None):
    """Load and write color image from local disk to the panel using RLE.

    RLE (Run Length Encoding) is used to apply non-lossy compression to the
    image data to reduce the transfer time over the UART.

    Args:
      image_path:
        str, path to an image file
      grayscale_clip:
        int, clip the pixels to this value 0-255(clip not scale)

    Returns:
      None

    Raises:
      FileNotFoundError: if image_path not found.

    """
    if not os.path.exists(image_path):
      raise FileNotFoundError(image_path)
    img = cv2.imread(image_path)

    if grayscale_clip is not None:
      if grayscale_clip > 255: grayscale_clip = 255
      img = np.clip(img, 0, grayscale_clip)
    # image order required appears to be RGB (based on inspection)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    rle_msg = self._color_image_to_rle_bytes(img)
    flow_len = len(rle_msg).to_bytes(4, "big")
    msg = (JbdControlMsg.write_color_image_rle.value + flow_len +
           b'\x00\x00\x00' + CMD_SUFFIX)
    chk = self._calc_xor_checksum(rle_msg)
    msg = msg + rle_msg + chk
    self.logger.debug("msg number of bytes = %s", len(msg))
    self.query(msg, response_delay=2)

  def _color_image_to_rle_bytes(self, image):
    """ Converts a color image to an RLE encoded byte array.

    Args:
      image:
        cv2 image data matrix, RGB order

    Returns:
      bytearray, RLE encoded image

    Raises:
      None
    """
    encoded_msg = bytearray()
    ch_data = [bytearray()] * 3
    ch_len = [bytearray()] * 3
    for i in range(3):
      ch_data[i] = self._rle_encode(bytearray(image[:,:,i].flatten()))
      ch_len[i] = len(ch_data[i]).to_bytes(4, "big")
    for i in range(3):
      encoded_msg.extend(ch_len[i])
    for i in range(3):
      encoded_msg.extend(ch_data[i])
    return encoded_msg

  def _rle_encode(self, in_msg):
    """ Encodes an input byte array with RLE (Run Length Encoding).

    Args:
      in_msg:
        bytearray, data to be encoded

    Returns:
       bytearray, RLE encoded

    Raises:
      None
    """
    encoded_msg = bytearray()
    i = 0
    j = 0
    while i < len(in_msg):
      length = min(256, len(in_msg) - i + 1)
      for j in range(1, length):
        if i + j >= len(in_msg):
          break
        elif in_msg[i + j] != in_msg[i]:
          break
        else:
          pass
      encoded_msg.append(in_msg[i])
      encoded_msg.append(j)
      i = i + j
    return encoded_msg
