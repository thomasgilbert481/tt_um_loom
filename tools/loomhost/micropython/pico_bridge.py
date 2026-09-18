# pico_bridge.py: USB-serial to SPI bridge for the Loom host port.
# SPDX-License-Identifier: Apache-2.0
#
# Runs under MicroPython on a Raspberry Pi Pico (RP2040). Copy it to the Pico
# as main.py. Line protocol (tools/loomhost/serial_transport.py):
#   H          -> LOOMBRIDGE 1
#   X <hex>    -> R <hex>      one transaction, CS_n low around it
#   I          -> Q 0|1        HOST_IRQ level
#   S <hz>     -> OK           SCK frequency (Loom needs SCK <= clk/8)
# Wiring (SPI0, same GPIO numbers as the Tiny Tapeout demo board):
#   GP17 -> HOST_CS_n ui_in[4]   GP18 -> HOST_SCK ui_in[5]
#   GP19 -> HOST_MOSI ui_in[6]   GP16 <- HOST_MISO uo_out[7]
#   GP20 <- HOST_IRQ uo_out[6]   and a common ground.
import sys
from machine import Pin, SPI

cs = Pin(17, Pin.OUT, value=1)
irq = Pin(20, Pin.IN)
spi = None


def open_spi(hz):
    global spi
    spi = SPI(0, baudrate=hz, polarity=0, phase=0, bits=8, firstbit=SPI.MSB,
              sck=Pin(18), mosi=Pin(19), miso=Pin(16))


def xfer(tx):
    rx = bytearray(len(tx))
    cs.value(0)
    try:
        spi.write_readinto(tx, rx)
    finally:
        cs.value(1)
    return rx


open_spi(1_000_000)
while True:
    line = sys.stdin.readline().strip()
    if not line:
        continue
    try:
        op = line[0]
        if op == "H":
            print("LOOMBRIDGE 1")
        elif op == "X":
            print("R " + xfer(bytes.fromhex(line[1:].strip())).hex())
        elif op == "I":
            print("Q %d" % irq.value())
        elif op == "S":
            open_spi(int(line[1:]))
            print("OK")
        else:
            print("E unknown command")
    except Exception as exc:  # report, keep serving
        print("E " + str(exc))
