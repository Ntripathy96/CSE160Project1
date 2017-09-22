#
# This class is automatically generated by mig. DO NOT EDIT THIS FILE.
# This class implements a Python interface to the 'CommandMsg'
# message type.
#

import tinyos.message.Message

# The default size of this message type in bytes.
DEFAULT_MESSAGE_SIZE = 28

# The Active Message type associated with this message.
AM_TYPE = 99

class CommandMsg(tinyos.message.Message.Message):
    # Create a new CommandMsg of size 28.
    def __init__(self, data="", addr=None, gid=None, base_offset=0, data_length=28):
        tinyos.message.Message.Message.__init__(self, data, addr, gid, base_offset, data_length)
        self.amTypeSet(AM_TYPE)
    
    # Get AM_TYPE
    def get_amType(cls):
        return AM_TYPE
    
    get_amType = classmethod(get_amType)
    
    #
    # Return a String representation of this message. Includes the
    # message type name and the non-indexed field values.
    #
    def __str__(self):
        s = "Message <CommandMsg> \n"
        try:
            s += "  [dest=0x%x]\n" % (self.get_dest())
        except:
            pass
        try:
            s += "  [id=0x%x]\n" % (self.get_id())
        except:
            pass
        try:
            s += "  [payload=";
            for i in range(0, 25):
                s += "0x%x " % (self.getElement_payload(i) & 0xff)
            s += "]\n";
        except:
            pass
        return s

    # Message-type-specific access methods appear below.

    #
    # Accessor methods for field: dest
    #   Field type: int
    #   Offset (bits): 0
    #   Size (bits): 16
    #

    #
    # Return whether the field 'dest' is signed (False).
    #
    def isSigned_dest(self):
        return False
    
    #
    # Return whether the field 'dest' is an array (False).
    #
    def isArray_dest(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'dest'
    #
    def offset_dest(self):
        return (0 / 8)
    
    #
    # Return the offset (in bits) of the field 'dest'
    #
    def offsetBits_dest(self):
        return 0
    
    #
    # Return the value (as a int) of the field 'dest'
    #
    def get_dest(self):
        return self.getUIntElement(self.offsetBits_dest(), 16, 1)
    
    #
    # Set the value of the field 'dest'
    #
    def set_dest(self, value):
        self.setUIntElement(self.offsetBits_dest(), 16, value, 1)
    
    #
    # Return the size, in bytes, of the field 'dest'
    #
    def size_dest(self):
        return (16 / 8)
    
    #
    # Return the size, in bits, of the field 'dest'
    #
    def sizeBits_dest(self):
        return 16
    
    #
    # Accessor methods for field: id
    #   Field type: short
    #   Offset (bits): 16
    #   Size (bits): 8
    #

    #
    # Return whether the field 'id' is signed (False).
    #
    def isSigned_id(self):
        return False
    
    #
    # Return whether the field 'id' is an array (False).
    #
    def isArray_id(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'id'
    #
    def offset_id(self):
        return (16 / 8)
    
    #
    # Return the offset (in bits) of the field 'id'
    #

    def offsetBits_seq(self):
        return 32
    
    #
    # Return the value (as a int) of the field 'seq'
    #
    def get_seq(self):
        return self.getUIntElement(self.offsetBits_seq(), 16, 1)
    
    #protocol
    # Set the value of the field 'seq'
    #
    def set_seq(self, value):
        self.setUIntElement(self.offsetBits_seq(), 16, value, 1)
    
    #
    def offsetBits_id(self):
        return 16
    #
    def offsetBits_protocol(self):
        return 56
    
    #
    # Return the value (as a short) of the field 'protocol'
    #
    def get_protocol(self):
        return self.getUIntElement(self.offsetBits_protocol(), 8, 1)
    
    #
    # Set the value of the field 'protocol'
    #
    def set_protocol(self, value):
        self.setUIntElement(self.offsetBits_protocol(), 8, value, 1)
    

    def offsetBits_TTL(self):
        return 48
    
    #
    # Return the value (as a short) of the field 'TTL'
    #
    def get_TTL(self):
        return self.getUIntElement(self.offsetBits_TTL(), 8, 1)
    
    #
    # Set the value of the field 'TTL'
    #
    def set_TTL(self, value):
        self.setUIntElement(self.offsetBits_TTL(), 8, value, 1)
    
    #
    # Return the value (as a short) of the field 'id'
    #
    def get_id(self):
        return self.getUIntElement(self.offsetBits_id(), 8, 1)
    
    #
    # Set the value of the field 'id'
    #
    def set_id(self, value):
        self.setUIntElement(self.offsetBits_id(), 8, value, 1)
    
    #
    # Return the size, in bytes, of the field 'id'
    #
    def size_id(self):
        return (8 / 8)
    
    #
    # Return the size, in bits, of the field 'id'
    #
    def sizeBits_id(self):
        return 8
    
    #
    # Accessor methods for field: payload
    #   Field type: short[]
    #   Offset (bits): 24
    #   Size of each element (bits): 8
    #

    #
    # Return whether the field 'payload' is signed (False).
    #
    def isSigned_payload(self):
        return False
    
    #
    # Return whether the field 'payload' is an array (True).
    #
    def isArray_payload(self):
        return True
    
    #
    # Return the offset (in bytes) of the field 'payload'
    #
    def offset_payload(self, index1):
        offset = 24
        if index1 < 0 or index1 >= 25:
            raise IndexError
        offset += 0 + index1 * 8
        return (offset / 8)
    
    #
    # Return the offset (in bits) of the field 'payload'
    #
    def offsetBits_payload(self, index1):
        offset = 24
        if index1 < 0 or index1 >= 25:
            raise IndexError
        offset += 0 + index1 * 8
        return offset
    
    #
    # Return the entire array 'payload' as a short[]
    #
    def get_payload(self):
        tmp = [None]*25
        for index0 in range (0, self.numElements_payload(0)):
                tmp[index0] = self.getElement_payload(index0)
        return tmp
    
    #
    # Set the contents of the array 'payload' from the given short[]
    #
    def set_payload(self, value):
        for index0 in range(0, len(value)):
            self.setElement_payload(index0, value[index0])

    #
    # Return an element (as a short) of the array 'payload'
    #
    def getElement_payload(self, index1):
        return self.getUIntElement(self.offsetBits_payload(index1), 8, 1)
    
    #
    # Set an element of the array 'payload'
    #
    def setElement_payload(self, index1, value):
        self.setUIntElement(self.offsetBits_payload(index1), 8, value, 1)
    
    #
    # Return the total size, in bytes, of the array 'payload'
    #
    def totalSize_payload(self):
        return (200 / 8)
    
    #
    # Return the total size, in bits, of the array 'payload'
    #
    def totalSizeBits_payload(self):
        return 200
    
    #
    # Return the size, in bytes, of each element of the array 'payload'
    #
    def elementSize_payload(self):
        return (8 / 8)
    
    #
    # Return the size, in bits, of each element of the array 'payload'
    #
    def elementSizeBits_payload(self):
        return 8
    
    def offset_src(self):
        return (16 / 8)
    
    #
    # Return the offset (in bits) of the field 'src'
    #
    def offsetBits_src(self):
        return 16
    
    #
    # Return the value (as a int) of the field 'src'
    #
    def get_src(self):
        return self.getUIntElement(self.offsetBits_src(), 16, 1)
     

    # Set the value of the field 'src'
    #
    def set_src(self, value):
        self.setUIntElement(self.offsetBits_src(), 16, value, 1)
    
    #
    #
    # Return the number of dimensions in the array 'payload'
    #
    def numDimensions_payload(self):
        return 1
    
    #
    # Return the number of elements in the array 'payload'
    #
    def numElements_payload():
        return 25
    
    #
    # Return the number of elements in the array 'payload'
    # for the given dimension.
    #
    def numElements_payload(self, dimension):
        array_dims = [ 25,  ]
        if dimension < 0 or dimension >= 1:
            raise IndexException
        if array_dims[dimension] == 0:
            raise IndexError
        return array_dims[dimension]
    
    #
    # Fill in the array 'payload' with a String
    #
    def setString_payload(self, s):
         l = len(s)
         for i in range(0, l):
             self.setElement_payload(i, ord(s[i]));
         self.setElement_payload(l, 0) #null terminate
    
    #
    # Read the array 'payload' as a String
    #
    def getString_payload(self):
        carr = "";
        for i in range(0, 4000):
            if self.getElement_payload(i) == chr(0):
                break
            carr += self.getElement_payload(i)
        return carr
    
