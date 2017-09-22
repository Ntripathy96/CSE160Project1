#ANDES Lab - University of California, Merced
#Author: UCM ANDES Lab
#$Author: abeltran2 $
#$LastChangedDate: 2014-08-31 16:06:26 -0700 (Sun, 31 Aug 2014) $
#! /usr/bin/python
import sys
from TOSSIM import *
from CommandMsg import *
from packet import *

class TestSim:
    
    # COMMAND TYPES
    CMD_PING = 0
    CMD_NEIGHBOR_DUMP = 1
    CMD_ROUTE_DUMP=3

    # CHANNELS - see includes/channels.h
    COMMAND_CHANNEL="command";
    GENERAL_CHANNEL="general";

    # Project 1
    NEIGHBOR_CHANNEL="neighbor";
    FLOODING_CHANNEL="flooding";

    # Project 2
    ROUTING_CHANNEL="routing";

    # Project 3
    TRANSPORT_CHANNEL="transport";

    # Personal Debuggin Channels for some of the additional models implemented.
    HASHMAP_CHANNEL="hashmap";

    # Initialize Vars
    numMote=0
    


    def __init__(self):
        self.t = Tossim([])
        self.r = self.t.radio()

        #Create a Command Packet
        self.msg = CommandMsg()
        #self.msg = pack()
        #self.msg.set_seq(0)
        #self.msg.set_TTL(15)
        self.msg.set_protocol(99)

        self.pkt = self.t.newPacket()
        self.pkt.setData(self.msg.data)
        self.pkt.setType(self.msg.get_amType())

    # Load a topo file and use it.
    def loadTopo(self, topoFile):
        print 'Creating Topo!'
        # Read topology file.
        topoFile = 'topo/'+topoFile
        f = open(topoFile, "r")
        self.numMote = int(f.readline());
        print 'Number of Motes', self.numMote
        for line in f:
            s = line.split()
            if s:
                print " ", s[0], " ", s[1], " ", s[2];
                self.r.add(int(s[0]), int(s[1]), float(s[2]))

    # Load a noise file and apply it.
    def loadNoise(self, noiseFile):
        if self.numMote == 0:
            print "Create a topo first"
            return;

        # Get and Create a Noise Model
        noiseFile = 'noise/'+noiseFile;
        noise = open(noiseFile, "r")
        for line in noise:
            str1 = line.strip()
            if str1:
                val = int(str1)
            for i in range(1, self.numMote+1):
                self.t.getNode(i).addNoiseTraceReading(val)

        for i in range(1, self.numMote+1):
            print "Creating noise model for ",i;
            self.t.getNode(i).createNoiseModel()

    def bootNode(self, nodeID):
        if self.numMote == 0:
            print "Create a topo first"
            return;
        self.t.getNode(nodeID).bootAtTime(1333*nodeID);

    def bootAll(self):
        i=0;
        for i in range(1, self.numMote+1):
            self.bootNode(i);

    def moteOff(self, nodeID):
        self.t.getNode(nodeID).turnOff();

    def moteOn(self, nodeID):
        self.t.getNode(nodeID).turnOn();

    def run(self, ticks):
        for i in range(ticks):
            self.t.runNextEvent()

    # Rough run time. tickPerSecond does not work.
    def runTime(self, amount):
        self.run(amount*1000)

    ##Create a Command Packet
    #msg = pack()
    

    #pkt = t.newPacket()
    #pkt.setData(msg.data)
    #pkt.setType(msg.get_amType())
    # Generic Command
    def sendCMD(self, ID, dest, payloadStr):
        
        
        self.msg.set_dest(dest);
        #self.msg.set_src(source);
        self.msg.set_id(ID);
        self.msg.setString_payload(payloadStr)

        
        self.pkt.setData(self.msg.data)
        self.pkt.setDestination(dest)
        self.pkt.deliver(dest, self.t.time()+5)
        #/*
        #self.msg.set_dest(dest);
        #self.msg.set_src(ID);
        #self.msg.setString_payload(payloadStr)
        #self.msg.set_protocol(99)
        


        #self.pkt.setData(self.msg.data)
        #self.pkt.setDestination(dest)
        #self.pkt.deliver(dest, self.t.time()+5)
        self.runTime(2)
 


    def ping(self, source, dest, msg):
        self.sendCMD(self.CMD_PING, source, "{0}{1}".format(chr(dest),msg));
        #self.sendCMD(self.CMD_PING, source, dest, msg);
       

    def neighborDMP(self, destination):
        self.sendCMD(self.CMD_NEIGHBOR_DUMP, destination, "neighbor command");

    def routeDMP(self, destination):
        self.sendCMD(self.CMD_ROUTE_DUMP, destination, "routing command");

    def addChannel(self, channelName, out=sys.stdout):
        print 'Adding Channel', channelName;
        self.t.addChannel(channelName, out);

def main():
    s = TestSim();
    s.runTime(10);
    s.loadTopo("long_line.topo");
    s.loadNoise("no_noise.txt");
    s.bootAll();
    s.addChannel(s.COMMAND_CHANNEL);
    s.addChannel(s.GENERAL_CHANNEL);
    s.addChannel(s.FLOODING_CHANNEL);

    s.runTime(20);
    s.ping(1, 8, "Hello, World");
    s.runTime(10);
    #s.ping(2,11, "Hi!");
    #s.runTime(20);
    #s.ping(14, 3, "FOUUUUND!");
    #s.runTime(20);
    #s.ping(1, 16, "HEEEEEEY!");
    #s.runTime(20);
if __name__ == '__main__':
    main()
