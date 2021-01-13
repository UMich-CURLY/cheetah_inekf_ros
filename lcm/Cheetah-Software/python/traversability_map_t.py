"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class traversability_map_t(object):
    __slots__ = ["map"]

    __typenames__ = ["int32_t"]

    __dimensions__ = [[100, 100]]

    def __init__(self):
        self.map = [ [ 0 for dim1 in range(100) ] for dim0 in range(100) ]

    def encode(self):
        buf = BytesIO()
        buf.write(traversability_map_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        for i0 in range(100):
            buf.write(struct.pack('>100i', *self.map[i0][:100]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != traversability_map_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return traversability_map_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = traversability_map_t()
        self.map = []
        for i0 in range(100):
            self.map.append(struct.unpack('>100i', buf.read(400)))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if traversability_map_t in parents: return 0
        tmphash = (0x65bb5bb541c18c9) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if traversability_map_t._packed_fingerprint is None:
            traversability_map_t._packed_fingerprint = struct.pack(">Q", traversability_map_t._get_hash_recursive([]))
        return traversability_map_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

