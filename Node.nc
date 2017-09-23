/*
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */
#include <Timer.h>
#include "includes/command.h"
#include "includes/packet.h"
#include "includes/CommandMsg.h"
#include "includes/sendInfo.h"
#include "includes/channels.h"

module Node{
   uses interface Boot;

   uses interface SplitControl as AMControl;
   uses interface Receive;

   uses interface SimpleSend as Sender;
   uses interface List<pack> as SeenPacketList; //use interface to create a seen packet list for each node
   uses interface CommandHandler;
}

implementation{
   pack sendPackage;
   uint16_t seqNumb = 0;
   // Prototypes
   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);
   bool findSeenPacket(pack *Package); //function for finding a packet from a node's seen packet list
   void pushToPacketList(pack Package); //push a seen packet onto a node's seen packet list

   event void Boot.booted(){
      call AMControl.start();

      dbg(GENERAL_CHANNEL, "Booted\n");
   }

   event void AMControl.startDone(error_t err){
      if(err == SUCCESS){
         dbg(GENERAL_CHANNEL, "Radio On\n");
      }else{
         //Retry until successful
         call AMControl.start();
      }
   }

   event void AMControl.stopDone(error_t err){}

   event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
      dbg(GENERAL_CHANNEL, "Packet Received\n");
      if(len==sizeof(pack)){
         pack* myMsg=(pack*) payload;
         dbg(GENERAL_CHANNEL, "Package Payload: %s\n", myMsg->payload);
         dbg(FLOODING_CHANNEL, "Msg_Package Sequence#: %d\n", myMsg->seq);
         

         if(myMsg->TTL == 0){ //meaning its TTL has run out and thus we should drop the packet

           dbg(FLOODING_CHANNEL,"TTL=0:Dropping packet seq #%d from %d to %d\n", myMsg->seq, myMsg->src, myMsg->dest); //notify what is happening

         }else if(findSeenPacket(myMsg)){//packet dropped if seen by node more than once
           dbg(FLOODING_CHANNEL,"ALREADY SEEN: Dropping packet seq #%d from %d to %d\n", myMsg->seq, myMsg->src, myMsg->dest); //notify what is happening

         }else if(TOS_NODE_ID == myMsg->dest){
             dbg(FLOODING_CHANNEL,"Packet from %d has arrived with Msg: %s and SEQ: %d\n", myMsg->src, myMsg->payload, myMsg->seq); //once again, notify what has happened 
             makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL-1,myMsg->protocol, seqNumb, (uint8_t *)myMsg->payload, sizeof(myMsg->payload));
             pushToPacketList(sendPackage); //push to seenpacketlist
             dbg(FLOODING_CHANNEL, "Sequence number before %d\n", seqNumb);
             seqNumb++;
             dbg(FLOODING_CHANNEL, "Sequence number after found %d\n", seqNumb);

         }else{ //packet does not belong to current node

            //resend same packet with TTL-1
            makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL-1,myMsg->protocol, myMsg->seq, (uint8_t *)myMsg->payload, sizeof(myMsg->payload));

            dbg(FLOODING_CHANNEL, "Recieved Message from %d meant for %d...Rebroadcasting\n", myMsg->src, myMsg->dest); //notify process
            pushToPacketList(sendPackage); //packet not meant for this node but we need to push into seenpacketlist
            dbg("Project1F", "SEQNUM %d, seq: %d. Rebroadcasting\n", seqNumb, myMsg->seq);
            //resend with broadcast address to move packet forward
            call Sender.send(sendPackage, AM_BROADCAST_ADDR);
         }

         return msg;
      }
      dbg(GENERAL_CHANNEL, "Unknown Packet Type %d\n", len);
      return msg;
   }

   bool findSeenPacket(pack *Package){
       uint16_t packetListSize = call SeenPacketList.size();
       uint16_t i = 0;
       pack packetMatcher; //use to try to find match

       for(i = 0; i < packetListSize; i++){ //traverse thru SeenPacketList
           packetMatcher = call SeenPacketList.get(i);
           if(packetMatcher.src == Package->src && packetMatcher.dest == Package->dest){
               return TRUE; //packet is found in SeenPacketList
           }
       }
       return FALSE; //packet not in SeenPacketList so we need to add it 
   }

   void pushToPacketList(pack Package){ //pushes a packet to back of SeenPacketList
       if(call SeenPacketList.isFull()){ //SeenPacketList is full so lets drop the first packet ever seen
            call SeenPacketList.popfront();
       }
       //add Package
       call SeenPacketList.pushback(Package);
   }

   event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
      dbg(GENERAL_CHANNEL, "PING EVENT \n");
      dbg(FLOODING_CHANNEL, "PING_Sequence number before %d\n", seqNumb);
      makePack(&sendPackage, TOS_NODE_ID, destination, 15, 0, seqNumb, payload, PACKET_MAX_PAYLOAD_SIZE);
      //seqNumb++;
      call Sender.send(sendPackage, AM_BROADCAST_ADDR);
      
   }
   
   event void CommandHandler.printNeighbors(){}

   event void CommandHandler.printRouteTable(){}

   event void CommandHandler.printLinkState(){}

   event void CommandHandler.printDistanceVector(){}

   event void CommandHandler.setTestServer(){}

   event void CommandHandler.setTestClient(){}

   event void CommandHandler.setAppServer(){}

   event void CommandHandler.setAppClient(){}

   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t protocol, uint16_t seq, uint8_t* payload, uint8_t length){
      Package->src = src;
      Package->dest = dest;
      Package->TTL = TTL;
      Package->seq = seq;
      Package->protocol = protocol;
      memcpy(Package->payload, payload, length);
   }
}
