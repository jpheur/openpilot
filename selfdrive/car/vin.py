#!/usr/bin/env python3
import cereal.messaging as messaging
from selfdrive.boardd.boardd import can_list_to_can_capnp

VIN_UNKNOWN = "0" * 17

# sanity checks on response messages from vin query
def is_vin_response_valid(can_dat, step, cnt):
  if len(can_dat) != 8:
    # ISO-TP meesages are all 8 bytes
    return False

  if step == 0:
    # VIN does not fit in a single message and it's 20 bytes of data
    if can_dat[0] != 0x10 or can_dat[1] != 0x14:
       return False

  if step == 1 and cnt == 0:
    # first response after a CONTINUE query is sent
    if can_dat[0] != 0x21:
       return False

  if step == 1 and cnt == 1:
    # second response after a CONTINUE query is sent
    if can_dat[0] != 0x22:
       return False

  return True


class VinQuery():
  def __init__(self, bus):
    self.bus = bus
    # works on standard 11-bit addresses for diagnostic. Tested on Toyota and Subaru;
    # Honda uses the extended 29-bit addresses, and unfortunately only works from OBDII
    self.query_ext_msgs = [[0x18DB33F1, 0, b'\x02\x09\x02'.ljust(8, b"\x00"), bus],
                           [0x18DA10f1, 0, b'\x30'.ljust(8, b"\x00"), bus]]
    self.query_nor_msgs = [[0x7df, 0, b'\x02\x09\x02'.ljust(8, b"\x00"), bus],
                           [0x7e0, 0, b'\x30'.ljust(8, b"\x00"), bus]]

    self.cnts = [1, 2]  # number of messages to wait for at each iteration
    self.step = 0
    self.cnt = 0
    self.responded = False
    self.never_responded = True
    self.dat = b""
    self.got_vin = False
    self.passive_frag_1, self.passive_frag_2, self.passive_frag_3 = None, None, None
    self.vin = VIN_UNKNOWN

  def check_active_query_response(self, msg):
    # have we got a VIN query response?
    if msg.src == self.bus and msg.address in [0x18daf110, 0x7e8]:
      self.never_responded = False
      # basic sanity checks on ISO-TP response
      if is_vin_response_valid(msg.dat, self.step, self.cnt):
        self.dat += bytes(msg.dat[2:]) if self.step == 0 else bytes(msg.dat[1:])
        self.cnt += 1
        if self.cnt == self.cnts[self.step]:
          self.responded = True
          self.step += 1
    if self.step == len(self.cnts):
      self.got_vin = True

  def check_passive_broadcast(self, msg):
    # Cheese-Tastic(TM) handler of Volkswagen PQ/MQB Component Protection VIN
    # mux messages. Community port only at this time; no particular guarantee
    # these messages aren't used on other vehicles for other purposes.
    if msg.address in [0x5D2, 0x6B4]:
      if msg.dat[0] == 0x0:
        self.passive_frag_1 = msg.dat[5:]
      elif msg.dat[0] == 0x1:
        self.passive_frag_2 = msg.dat[1:]
      elif msg.dat[0] == 0x2:
        self.passive_frag_3 = msg.dat[1:]
      if self.passive_frag_1 and self.passive_frag_2 and self.passive_frag_3:
        vin_tmp = self.passive_frag_1 + self.passive_frag_2 + self.passive_frag_3
        self.vin = vin_tmp.decode('utf8')
        self.got_vin = True

  def send_query(self, sendcan):
    # keep sending VIN query if ECU isn't responding.
    # sendcan is probably not ready due to the zmq slow joiner syndrome
    if self.never_responded or (self.responded and not self.got_vin):
      sendcan.send(can_list_to_can_capnp([self.query_ext_msgs[self.step]], msgtype='sendcan'))
      sendcan.send(can_list_to_can_capnp([self.query_nor_msgs[self.step]], msgtype='sendcan'))
      self.responded = False
      self.cnt = 0

  def get_vin(self):
    if self.got_vin and self.responded is True:
      try:
        self.vin = self.dat[3:].decode('utf8')
      except UnicodeDecodeError:
        pass  # have seen unexpected non-unicode characters on active query
    return self.vin


def get_vin(logcan, sendcan, bus, query_time=1.):
  vin_query = VinQuery(bus)
  frame = 0

  # 1s max of VIN query time
  while frame < query_time * 100 and not vin_query.got_vin:
    a = messaging.get_one_can(logcan)

    for can in a.can:
      vin_query.check_active_query_response(can)
      vin_query.check_passive_broadcast(can)
      if vin_query.got_vin:
        break

    vin_query.send_query(sendcan)
    frame += 1

  return vin_query.get_vin()


if __name__ == "__main__":
  logcan = messaging.sub_sock('can')
  sendcan = messaging.pub_sock('sendcan')
  print(get_vin(logcan, sendcan, 0))
