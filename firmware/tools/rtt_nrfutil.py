#!/usr/bin/env python3
"""RTT-Log vom R1A lesen — über `nrfutil device x-read --direct` statt J-Link.
Hintergrund (2026-09-16, siehe HANDOFF.md): J-Link Commander/RTTLogger/pylink kommen
auf dem R1A nicht sauber an den Core (Attach scheitert -> connect-under-reset ->
Lockup/AP-Protect). nrfutil liest korrekt; `--direct` liest OHNE Reset.
ACHTUNG: jedes `x-write` (auch --direct) resettet das Board -> Lesen ist rein
passiv: RdOff wird NICHT nachgezogen, Offset wird lokal gefuehrt. Down-Channel
(Shell-Eingabe) funktioniert deshalb nur mit Reset -> `send_down` nur bewusst nutzen.
Aufruf: rtt_nrfutil.py <sekunden> [text-für-down-channel]   (python3 + nrfutil)"""
import subprocess, struct, sys, time, re, os
SN=os.environ.get("RTT_SN","1051297775"); CB=int(os.environ.get("RTT_CB","0x2000c980"),16); TMP="/tmp/rtt_chunk.bin"
def rd(addr, n):
    n4 = (n+3)//4*4
    p = subprocess.run(["nrfutil","device","x-read","--address",hex(addr),"--bytes",str(n4),
                        "--direct","--serial-number",SN],check=True,capture_output=True,text=True)
    out=b""
    for line in p.stdout.splitlines():
        m=re.match(r"\s*0x[0-9A-Fa-f]+:\s+((?:[0-9A-Fa-f]{8}\s*)+)",line)
        if m:
            for w in m.group(1).split(): out+=struct.pack("<I",int(w,16))
    return out[:n]
def wr32(addr, val):
    subprocess.run(["nrfutil","device","x-write","--address",hex(addr),"--value",hex(val),"--direct",
                    "--serial-number",SN],check=True,capture_output=True)
hdr = rd(CB, 24)
assert hdr[:10]==b"SEGGER RTT", hdr
nup, ndown = struct.unpack_from("<ii", hdr, 16)
up0 = CB+24; down0 = CB+24+24*nup
def buf(addr):
    d = rd(addr, 24); name,pbuf,size,wr,rdo,flags = struct.unpack("<IIIIII", d); return pbuf,size,wr,rdo
_rd = None
def read_up():
    global _rd
    pbuf,size,wr,rdo = buf(up0)
    if _rd is None: _rd = rdo          # ab Firmware-RdOff starten (ganzer Puffer seit Boot)
    if wr==_rd: return b""
    if wr>_rd: data = rd(pbuf+_rd, wr-_rd)
    else: data = rd(pbuf+_rd, size-_rd) + (rd(pbuf, wr) if wr else b"")
    _rd = wr                            # nur lokal, kein x-write (wuerde resetten)
    return data
def send_down(text):
    pbuf,size,wr,rdo = buf(down0)
    data = text.encode()+b"\n"
    for i in range(0,len(data),4):
        w = data[i:i+4].ljust(4,b"\x00")
        wr32(pbuf+((wr+i)%size), struct.unpack("<I",w)[0])
    wr32(down0+12, (wr+len(data))%size)   # WrOff
secs=float(sys.argv[1]); out=b""
if len(sys.argv)>2: send_down(sys.argv[2]); print(">>> sent:", sys.argv[2])
t0=time.time()
while time.time()-t0<secs:
    d=read_up()
    if d: out+=d; sys.stdout.write(re.sub(r'\x1b\[[0-9;]*m','',d.decode('utf-8','replace'))); sys.stdout.flush()
    else: time.sleep(0.3)
print("\n--- %d bytes" % len(out))
