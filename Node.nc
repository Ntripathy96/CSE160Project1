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

typedef nx_struct neighbor {
    nx_uint16_t Node;
    nx_uint8_t Life;
}neighbor;

module Node{
   uses interface Boot;

   uses interface SplitControl as AMControl;
   uses interface Receive;

   uses interface SimpleSend as Sender;
   uses interface CommandHandler;

   uses interface List<pack> as SeenPacketList; //use interface to create a seen packet list for each node
   uses interface List<neighbor*> as ListOfNeighbors;
   uses interface Pool<neighbor> as PoolOfNeighbors;
   uses interface Timer<TMilli> as Timer1; //uses timer to create periodic firing on neighbordiscovery and to not overload the network


}

implementation{
   pack sendPackage;
   uint16_t seqNumb = 0;
   
   // Prototypes
   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);
   bool findSeenPacket(pack *Package); //function for finding a packet from a node's seen packet list
   void pushToPacketList(pack Package); //push a seen packet onto a node's seen packet list
   void neighborDiscovery(); //find a nodes neighbors
   void printNeighbors(); //print a nodes neighbor list



   event void Boot.booted(){
      call AMControl.start();

      dbg(GENERAL_CHANNEL, "Booted\n");

      call Timer1.startPeriodicAt(1,1500);
      dbg(NEIGHBOR_CHANNEL,"Timer started");
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
   
   //fired() event for Timer1
   event void Timer1.fired(){
       neighborDiscovery();
   }
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
            //dbg(FLOODING_CHANNEL, "SEQNUM %d, seq: %d. Rebroadcasting\n", seqNumb, myMsg->seq);

         }else if(TOS_NODE_ID == myMsg->dest){
             dbg(FLOODING_CHANNEL,"Packet from %d has arrived with Msg: %s and SEQ: %d\n", myMsg->src, myMsg->payload, myMsg->seq); //once again, notify what has happened 
             
             //dbg(FLOODING_CHANNEL, "Sequence number before %d\n", seqNumb);
             //seqNumb++;
             //dbg(FLOODING_CHANNEL, "Sequence number after found %d\n", seqNumb);
             //makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL-1,myMsg->protocol, myMsg->seq, (uint8_t *)myMsg->payload, sizeof(myMsg->payload));
             //we wouldnt want to push protocol_cmd into the nodes list because this will break multiple pings being able to be sent by same node
             
             if(myMsg->protocol != PROTOCOL_CMD){
                pushToPacketList(*myMsg); //push to seenpacketlist
             }

             switch(myMsg->protocol){

                uint8_t createMsg[PACKET_MAX_PAYLOAD_SIZE];
                uint16_t destOff;

                case PROTOCOL_PING:
                    dbg(NEIGHBOR_CHANNEL, "Sending Ping Reply to %d\n", myMsg->src);

                    //make the package with myMsg->src as dest
                    makePack(&sendPackage, TOS_NODE_ID, myMsg->src, MAX_TTL,PROTOCOL_PINGREPLY,seqNumb,(uint8_t *)myMsg->payload, sizeof(myMsg->payload));
                    seqNumb++; //increase seqNumb
                    dbg(NEIGHBOR_CHANNEL,"Increase SeqNum: %d\n", seqNumb);

                    //push packet onto ListOfNeighbors
                    pushToPacketList(sendPackage);
                    call Sender.send(sendPackage, AM_BROADCAST_ADDR);
                    break;
                
                case PROTOCOL_PINGREPLY:
                    dbg(NEIGHBOR_CHANNEL,"Ping Reply Incoming from %d \n", myMsg->src);
                    break;
                
                case PROTOCOL_CMD:
                    switch(getCMD((uint8_t *)&myMsg->payload,sizeof(myMsg->payload))){

                        case CMD_NEIGHBOR_DUMP:
                            printNeighbors();
                            break;
                        
                        case CMD_PING:
                            memcpy(&createMsg, (myMsg->payload) + CMD_LENGTH+1, sizeof(myMsg->payload) - CMD_LENGTH+1);
						    memcpy(&destOff, (myMsg->payload)+ CMD_LENGTH, sizeof(uint8_t));
						    makePack(&sendPackage, TOS_NODE_ID, (destOff-48)&(0x00FF),MAX_TTL, PROTOCOL_PING, seqNumb, (uint8_t *)createMsg, sizeof(createMsg));	
						    seqNumb++;
						    //Push the packet we want to send into our seen/sent list
						    pushToPacketList(sendPackage);
						    call Sender.send(sendPackage, AM_BROADCAST_ADDR);
						    break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
             
        }else if(AM_BROADCAST_ADDR == myMsg->dest){//tell nodes what to do when recieving a packet from broadcast addr

            bool FOUND;
            uint16_t i =0, size;
            neighbor * Neighbor, *neighbor_ptr;

            //what protocol does this message come with
            switch(myMsg->protocol){

                case PROTOCOL_PING: //PROTOCOL_PING = 0
                    dbg(GENERAL_CHANNEL, "MYMSG->PROTOCOL %d\n", myMsg->protocol);
                    //we recieve a protocol ping, we must send packet back to sender so they can discover a neighbor
                    dbg(NEIGHBOR_CHANNEL, "NODE %d Received Protocol Ping from %d\n",TOS_NODE_ID,myMsg->src);

                    //create a package with protocol PINGREPLY FOR myMsg->src from TOS_NODE_ID
                    //PROTOCOL_PINGREPLY = 1
                    dbg(NEIGHBOR_CHANNEL,"AMMMS = %d\n", AM_BROADCAST_ADDR);
                    dbg(NEIGHBOR_CHANNEL,"size = %d\n", sizeof(myMsg->payload));
                    //dbg(NEIGHBOR_CHANNEL,"PAYLOAD = %d\n", myMsg->payload);
                    //dbg(NEIGHBOR_CHANNEL,"TOS_NODE_ID = %d\n", TOS_NODE_ID);
                    //dbg(NEIGHBOR_CHANNEL,"TTL = %d\n", myMsg->TTL-1);
                    //dbg(NEIGHBOR_CHANNEL,"PROTOCOL = %d\n", PROTOCOL_PINGREPLY);
                    //dbg(NEIGHBOR_CHANNEL,"SEQ = %d\n", myMsg->seq);
                    dbg(NEIGHBOR_CHANNEL,"AMMMS = %d\n", AM_BROADCAST_ADDR);
                    
                    


                    makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, myMsg->TTL-1,PROTOCOL_PINGREPLY, myMsg->seq, (uint8_t *)myMsg->payload, sizeof(myMsg->payload));
                    dbg(NEIGHBOR_CHANNEL, "inbetween = %s\n", sendPackage.protocol);
                    pushToPacketList(sendPackage); //push to our seen list

                    dbg(NEIGHBOR_CHANNEL, "New PROTOCOL AFTER PINGPROTOCOL = %s\n", sendPackage.protocol);
                    call Sender.send(sendPackage, myMsg->src); //send back to sender with PINGREPLY Protocol
                    break;
                
                case PROTOCOL_PINGREPLY:
                    //we got a ping reply from a neighbor so we need to update that neighbors life to 0 again because we have seen it again
                    dbg(NEIGHBOR_CHANNEL, "Recieved PINGREPLY from %d\n", myMsg->src);
                    FOUND = FALSE; //IF FOUND, we switch to TRUE
                    size = call ListOfNeighbors.size();

                    for(i = 0; i < size; i++){
                        neighbor_ptr = call ListOfNeighbors.get(i);
                        if(neighbor_ptr->Node == myMsg->src){
                            //found neighbor in list, reset life
                            dbg(NEIGHBOR_CHANNEL, "Node %d found in neighbor list\n", myMsg->src);
                            neighbor_ptr->Life = 0;
                            FOUND = TRUE;
                            break;
                        }
                    }

                    //if the neighbor is not found it means it is a new neighbor to the node and thus we must add it onto the list by calling an allocation pool for memory PoolOfNeighbors
                    if(!FOUND){
                        dbg(NEIGHBOR_CHANNEL, "NEW Neighbor: %d added to neighbor list\n", myMsg->src);
                        Neighbor = call PoolOfNeighbors.get(); //get New Neighbor
                        Neighbor->Node = myMsg->src; //add node source
                        Neighbor->Life = 0; //reset life
                        call ListOfNeighbors.pushback(Neighbor); //put into list 

                    }
                    break;
                default:
                    break;



                
            }
             
             
             
         }else{ //packet does not belong to current node

            //resend same packet with TTL-1
            makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL-1,myMsg->protocol, myMsg->seq, (uint8_t *)myMsg->payload, sizeof(myMsg->payload));

            dbg(FLOODING_CHANNEL, "Recieved Message from %d meant for %d...Rebroadcasting\n", myMsg->src, myMsg->dest); //notify process
            pushToPacketList(sendPackage); //packet not meant for this node but we need to push into seenpacketlist
            //dbg(FLOODING_CHANNEL, "SEQNUM %d, seq: %d. Rebroadcasting\n", seqNumb, myMsg->seq);
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
           if(packetMatcher.src == Package->src && packetMatcher.dest == Package->dest && packetMatcher.seq == Package->seq){
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


   void neighborDiscovery() {
		pack Package;
		char* message;
		
		dbg(NEIGHBOR_CHANNEL, "Neighbor Discovery: checking node %d list for its neighbors\n", TOS_NODE_ID);
		if(!call ListOfNeighbors.isEmpty()) {
			uint16_t size = call ListOfNeighbors.size();
			uint16_t i = 0;
			uint16_t life = 0;
			neighbor* myNeighbor;
			neighbor* tempNeighbor;
			//Increase Life of the ListOfNeighbors if not seen, every 5 pings a neighbor isnt seen, we are going to remove it
			for(i = 0; i < size; i++) {
				tempNeighbor = call ListOfNeighbors.get(i);
				tempNeighbor->Life++;
			}
			//Check if neighbors havent been called or seen in a while, if 5 pings occur and neighbor is not hear from, we drop it
			for(i = 0; i < size; i++) {
				tempNeighbor = call ListOfNeighbors.get(i);
				life = tempNeighbor->Life;
				if(life > 5) {
					myNeighbor = call ListOfNeighbors.remove(i);
					dbg(NEIGHBOR_CHANNEL, "Node %d life has expired dropping from NODE %d list\n", myNeighbor->Node, TOS_NODE_ID);
					call PoolOfNeighbors.put(myNeighbor);
					i--;
					size--;
				}
			}
		}
        //after increasing neighbors life and dropping any neighbor with expiring life, we can now ping ListOfNeighbors
		//using an arbitrary message, we are just trying to find the neighbors, not send real messages
		message = "addOn\n";
		makePack(&Package, TOS_NODE_ID, AM_BROADCAST_ADDR, 2, PROTOCOL_PING, 1, (uint8_t*) message, (uint8_t) sizeof(message));

		pushToPacketList(Package);
		call Sender.send(Package, AM_BROADCAST_ADDR);
	}



   void printNeighbors() {
		uint16_t i, size;
		size = call ListOfNeighbors.size();
		//Print out ListOfNeighbors after updating
		if(size == 0) {
			dbg(NEIGHBOR_CHANNEL, "No Neighbors found for %d\n", TOS_NODE_ID);
		} else {
			dbg(NEIGHBOR_CHANNEL, "Updated Neighbors. Dumping new neighbor list of size %d for Node %d\n", size, TOS_NODE_ID);
			for(i = 0; i < size; i++) {
				neighbor* myNeighbor = call ListOfNeighbors.get(i);
				dbg(NEIGHBOR_CHANNEL, "FOUND: Neighbor: %d, Life: %d\n", myNeighbor->Node, myNeighbor->Life);
			}
		}
	}

   event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
      dbg(GENERAL_CHANNEL, "PING EVENT \n");
      //dbg(FLOODING_CHANNEL, "PING_Sequence number before %d\n", seqNumb);
      //dbg(FLOODING_CHANNEL, "sendPackage.seq number before %d\n", sendPackage.seq);
      //sendPackage.seq+1 increases seq# by 1 to give each packet an unique seq#
      seqNumb = sendPackage.seq + 1;
      //dbg(FLOODING_CHANNEL, "PING_Sequence number after %d\n", seqNumb);
      makePack(&sendPackage, TOS_NODE_ID, destination, 15, 0, seqNumb, payload, PACKET_MAX_PAYLOAD_SIZE);
      
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
