import simpy
import random
import numpy
import math

#########################################control variables used in the code########################################################################################################
N=300																#the number of smart meters that are communicating to the Base stations
CollisionCount=0
NoOfRetransmission=0
keepAliveCount=0													#keepAlive Packet count
NormalCount=0 														#normal packet count
AlarmCount=0 														#alarm packet Count
NoOfRetransmissionAlarm=0 											#total number of Alarm packet that got retransmitted
NoOfRetransmissionKeepAlive=0 										# total number of keepAlive packet that got retransmitted
NoOfRetransmissionNormal=0 											#total number of Normal packet that got retranmsitted
#STATUSA =False 														#this global variable is used for collision detection at Base Station 1
#STATUSB =False 														#this global variable is used for collision detection at Base Station 2
TotalTime=0 														#this variable holds the sum of delay time of all the packets that are generated from smart meters, to caluclate the average delay
#LoopBackPeriod = 5 #ACK response time
pthres=0.75															#probability of threshold for the Base station to detect the packet....based on the signal receiving capacity of the Base station
#BS1_ACK_send_STATUS=False #this variable is used to prevent deduplication of ACK. if Base station has succesfully transmitted an ACK den BS2 should not transmit
ACK_STATUS=numpy.full(N,False,dtype=bool) 							#this array holds ACK status of each packet that is ACKed by the Base station
ACK_ID=numpy.zeros(N)												#this array will hold the IDs of the Smart mter whose packet got ACKed..it is used in run() of PacketSink and Timer() of PacketGenerator
SEQUENCE=0															#this variable stores the sequence number of the packet ACKed, so as to verify that at the transmitter
TYPE = ["" for x in range(N)]										#this is an array that stores the type of packet ACKed by the Base station. Goto run() of PacketSink to see its usage
#ACK_TIME=numpy.zeros(N,dtype=numpy.float)
ACK_PACKET_ID=0														#this variable holds the ID of the smart meter whose packet  got ACKed.
PKT_ACKED_SUCCESFULLY=0
k=1																	#this variable is used to use same channel for broadcasting at a Smart meter...
collision_check=0																	#goto __init__() of class PacketGenerator to see its implementation
x=1																	#this variable is used to logically prevent duplicate maintenance of packets in the smart meter queue....
																	#goto run() of Class PacketGenerator to see its usage
cache=0																#this variable is used to use same channel for broadcasting at a Smart meter...
																	#goto __init__() of class PacketGenerator to see its implementation
print_control=0																		
######################################## Packet Class ##############################################################################################################################
class Packet(object):
    """ A very simple class that represents a packet.
        This packet will run through a queue at a switch output port.
        We use a float to represent the size of the packet in bytes so that
        we can compare to ideal M/M/1 queues.

        Parameters
        ----------
        time : float
            the time the packet arrives at the output queue.
        size : float
            the size of the packet in bytes
        id : int
            an identifier for the packet
		seq_no: int
			it is the sequence number of the packet used for retransmission
        src, dst : int
            identifiers for source and destination
        flow_id : int
            small integer that can be used to identify a flow
		channel : int
			channel id used by the nodes to transmit that particular packet
		bitrate : int
			for uplink we take as 250 bits/sec and downlink we take 600 bits/sec
		prob_rec_BS1,prob_rec_BS2 : float
			it is the probabibilty of successful reception at two Base stations respectively
		type : string
			defines the type of packet...In our case we are considering the Normal, keepAlive, Alarm and ACK packets
		retransmitted : boolean
			defines whether the packet is a retransmitted or a non retransmitted packet
		retrans_no :int
			number of times the packet is retransmitted
    """
    def __init__(self, time, size, id,seq_no,src="a", dst="z", flow_id=0, channel=10,bitrate=0,prob_rec_BS1=1,type="ACK",retransmitted=False,retrans_no=0):
        self.time = time
        self.size = size
        self.id = id
        self.src = src
        self.dst = dst
        self.flow_id = flow_id
        #self.channel=channel
        global k,cache 
        if(self.src==1 or self.src==2):
            self.channel=channel
        elif(self.src >2  and (self.size ==283.0 or self.size ==772.0)):
            self.channel=1
        elif(self.src >2  and (self.size !=283.0 or self.size !=772.0)): #if household packet
            k+=1
            cache=random.randint(2,channel)
            self.channel=cache
      
        self.bitrate=bitrate
        self.prob_rec_BS1=prob_rec_BS1
        #self.prob_rec_BS2=prob_rec_BS2
        self.type=type
        self.retransmitted=retransmitted
        self.seq_no=seq_no
        if(self.retransmitted == True):
            self.retrans_no=retrans_no
        else:
            self.retrans_no=0    
        self.ACKed=False
    def __repr__(self):
        if(self.src>2):
            #print"*****************************************************************************************"
            return "UPLINK {} Packet : id: {}, src: {},seq_no={}, time: {}, size: {} ,channel:{}, bitrate:{}, Prob_R at BS1:{}, retransmitted:{} ".\
                format(self.type,self.id, self.src,self.seq_no, self.time, self.size,self.channel,self.bitrate,self.prob_rec_BS1,self.retransmitted)
        else:
            #print"*****************************************************************************************"
            return "DOWNLINK Packet : id: {}, src: {},seq_no={}, time: {}, size: {} ,channel:{}, bitrate:{}".\
                format(self.id, self.src,self.seq_no ,self.time, self.size,self.channel,self.bitrate)
    
	#this function increments the retransmission for a packet
    def noOfRetransmission(self):
        self.retrans_no=self.retrans_no+1
        return self.retrans_no

######################################## PacketGenerator Class ##############################################################################################################################
class PacketGenerator(object):
    """ Generates packets with given inter-arrival time distribution.
        Set the "out" member variable to the entity to receive the packet.

        Parameters
        ----------
        env : simpy.Environment
            the simulation environment
        adist : function
            a no parameter function that returns the successive inter-arrival times of the packets
        sdist : function
            a no parameter function that returns the successive sizes of the packets
		
        initial_delay : number
            Starts generation after an initial delay. Default = 0
        finish : number
            Stops generation at the finish time. Default is infinite
        pktType:either keepAlive,Alarm or Normal or ACK

    """
    def __init__(self, env, id,  adist, sdist, channel, bitrate,prob_rec_BS1,seq_no,type="ACK",retransmitted=False,initial_delay=0, finish=float("inf"), flow_id=0,packetRecv=False):
        self.id = id
        self.env = env
        self.adist = adist
        self.sdist = sdist
        self.initial_delay = initial_delay
        self.finish = finish
        self.out = None
        self.type=type
        self.retransmitted=retransmitted
        self.packets_sent = 0
        self.action = env.process(self.run())  # starts the run() method as a SimPy process
        self.action = env.process(self.Timer()) #this is another timer () method that runs concurrently with the run() method...it is used to probe the received ACK at the Smart meters
        self.flow_id = flow_id
        self.ContentionWindow=4
        self.channel=channel
        #when the id =1, i.e. Base station ID then the same channel with which the packet was received by the Base station 
        #would be used for ACK transmission 
        #In the else part for Smartmeters it is random channel selection
        #self.packetRecv=packetRecv
        self.prob_rec_BS1=prob_rec_BS1
        #self.prob_rec_BS2=prob_rec_BS2
        self.bitrate=bitrate
        
        if(self.id==1):
            self.seq_no=seq_no #the sequence number used by the Base station here are the ACK number thats why they should be same as the seq number of the packet received at Base station
        else:
            self.seq_no=0 # A smart meter must initiates his packet generation with a sequence number of zero
            
        self.Queue=[] #for storing the packets transmitted...so if they are not ACKed in the retransmission timeout period, they can be transmitted again by fetching from queue
        self.Queue_Size=100#maximum size of a queue for a device would be 10

    def run(self):
        """The generator function used in simulations.
        """
        global x,keepAliveCount,NormalCount,AlarmCount
        yield self.env.timeout(self.initial_delay)
        while self.env.now < self.finish: #till infinite
            self.packets_sent += 1
            # wait for next transmission...yielding based on the type of packet generated as the ACK is conditioned we provide a constant rather than function based
            if (self.id==1 ): #sending the ACKs once for BS1 and BS2...that is why for packet_sent == 1 they just yield once... for packet_sent > 1, yield timeout is infinite.. that means they do not yield
                if(self.packets_sent==1):
                    yield self.env.timeout(self.adist)  #yielding for packet generated as ACK 
                    p = Packet(self.env.now, self.sdist, self.packets_sent, src=self.id,seq_no=self.seq_no, flow_id=self.flow_id,channel=self.channel,bitrate=self.bitrate,prob_rec_BS1=self.prob_rec_BS1,type=self.type)
                    self.out.put(p)
                else:#to make the Base station transmit one ACK for one reception of Data
                    yield self.env.timeout(float("inf")) #this is to prevent the continous transmission to ACK packets..when packets_sent=2,arrival time made infinite
            else:#this is the normal,keepalive and alarm packet generation at Smart meters 
                yield self.env.timeout(self.adist()) #yielding for other packet
                #here we need to implement the retransmission portion
                self.seq_no+=1
                if(self.seq_no>=4095):#wraps around at 4095 as 12 bits allocated to sequence counter
                    self.seq_no=0
                #first packet arrival....so send the packet and enqueue it
                #we should not yield twice....or the net yield gets added up....
                #transmit original packet
                p = Packet(self.env.now, self.sdist, self.packets_sent, src=self.id,seq_no=self.seq_no, flow_id=self.flow_id,channel=self.channel,bitrate=self.bitrate,prob_rec_BS1=self.prob_rec_BS1,type=self.type)
                self.out.put(p)
                if(self.type=="keepAlive"):
                    keepAliveCount+=1
                    #print "keepalive:%d"%keepAliveCount
                elif(self.type=="Normal"):
                    NormalCount+=1
                    print "Normal:%d"%NormalCount
                    self.enQueue(p)
                elif(self.type=="Alarm"):
                    AlarmCount+=1
                    print "Alarm:%d"%AlarmCount
                    self.enQueue(p)
                                                   
    def Timer(self):
        global ACK_ID,ACK_STATUS,SEQUENCE,TYPE,N,ACK_PACKET_ID,PKT_ACKED_SUCCESFULLY,NoOfRetransmissionAlarm,NoOfRetransmissionKeepAlive,NoOfRetransmissionNormal,TotalTime
        count_keepAlive=0
        count_Normal=0
        count_Alarm=0
        
        yield self.env.timeout(self.initial_delay)
        while self.env.now < self.finish:
            yield self.env.timeout(0.2)
            for i in range(len(self.Queue)): 
             f=len(self.Queue)-1-i
             #print "f for SM%d = %d"%(self.id,f)
	     #if current time - sent pkt time < 2 RTT i.e. packet gets acked before the retranmsission timeout i.e. before 2 RTT
             if(self.env.now-self.getDeparture(self.Queue[f].seq_no)<=2*((self.Queue[f].size*8/250)+(14*8/600)+5) and self.id>2):
                #print "current time%d wait time %d for %d wd seq no %d and 2*RTT %d"%(self.env.now,self.env.now-self.getDeparture(self.Queue[i].seq_no),self.id,self.Queue[i].seq_no,2.5*((self.Queue[i].size*8/250)+(14*8/600)))
                #self.display()
                for j in range(0,N): 
                    #print "ACK_STATUS%s and ACK_ID %d and Type %s and current type %s and time %0.4f and seq no %d" %(ACK_STATUS[j],ACK_ID[j],TYPE[j],self.type,self.env.now,self.Queue[f].seq_no)
                    if(ACK_STATUS[j]==True and ACK_ID[j]==self.id and TYPE[j]==self.Queue[f].type and self.Queue[f].seq_no==SEQUENCE-1 ):
                        print "if a packet is ACKed and id is %d and we deque seq no %d"%(self.id,SEQUENCE-1)
                        #self.display()
                        self.Queue[f].ACKed=True
                        print "if the packet %d sent at %f is acked %r  current time %0.4f delay %0.4f"%(f,self.Queue[f].time,self.Queue[f].ACKed,self.env.now,self.env.now-self.Queue[f].time) 
                        temp=self.env.now-self.Queue[f].time
                        TotalTime=TotalTime+temp
                        self.deQueue(SEQUENCE-1)
                        PKT_ACKED_SUCCESFULLY+=1
                        ACK_STATUS[j]=False
                        ACK_ID[j]=0 
                        count_keepAlive=0
                        count_Normal=0
                        count_Alarm=0
                        print"Total time : %f"%TotalTime
                        print"PKT_ACKED_SUCCESFULLY=%d"%PKT_ACKED_SUCCESFULLY
                        
                #if waiting for ACK time becomes greater than > 2RTT....RTT = size of transmitted packet * 8 /250(SM to BS) + mean loopback period i.e. 5 sec + 14 * 8/600 (BS to SM)
             elif ((self.env.now-self.getDeparture(self.Queue[f].seq_no))>=2*((self.Queue[f].size*8/250)+(14*8/600)+5)and self.id>2 ): 
            
				
				######################## Re Transmission for KEEPALIVE PACKET ####################################################################################################
				#count_keepAlive < 1 means we are taking a maximum of 1 retransmission for keep alive packet
		#print "Abhijeet!!*******entering the RETRANSMISSION block********"
		#print "ACK status of SM %d of type %s is %r and f=%d"%(self.id,self.Queue[f].type,self.Queue[f].ACKed,f)
		#print "Control Variables count_Normal=%d count_Alarm=%d "%(count_Normal,count_Alarm)
		#self.display()
                if(self.Queue[f].type=="keepAlive" and count_keepAlive<0  and self.Queue[f].ACKed==False ):
			BackOff=random.randint(0,self.ContentionWindow)
                        Retp1 = Packet(self.env.now+BackOff, self.Queue[f].size,count_keepAlive+1, src=self.id,seq_no=self.Queue[f].seq_no, flow_id=self.Queue[f].flow_id,channel=self.channel,bitrate=self.Queue[f].bitrate,prob_rec_BS1=self.Queue[f].prob_rec_BS1,type=self.Queue[f].type,retransmitted=True,retrans_no=count_keepAlive) 
                        count_keepAlive=Retp1.noOfRetransmission()
                        self.deQueue(self.Queue[f].seq_no)
                        self.enQueue(Retp1)
                        BS1_sink=PacketSink(self.env,1, rec_arrivals=True, absolute_arrivals=True, rec_waits=True,debug=True) 
                        BS1_sink.put(Retp1)
                        NoOfRetransmissionKeepAlive+=1
                        #print "No of Retransmission Keep Alive %d"%NoOfRetransmissionKeepAlive
						
				######################## Re Transmission for NORMAL AMI packets ####################################################################################################
				#count_Normal < 2 means we are taking a maximum of 2 retransmission for normal packet
                elif(self.Queue[f].type=="Normal" and count_Normal<2  and self.Queue[f].ACKed==False):
                        #BackOff=random.randint(0,self.ContentionWindow)*self.Queue[f].size*8/250
                        BackOff=random.randint(0,math.pow(self.ContentionWindow,count_Normal+1))*self.Queue[f].size*8/250 
                        temp2=self.env.now+BackOff-self.getDeparture(self.Queue[f].seq_no)
                        TotalTime=TotalTime+temp2
                        Retp1 = Packet(self.env.now+BackOff, self.Queue[f].size, count_Normal+1, src=self.id,seq_no=self.Queue[f].seq_no, flow_id=self.Queue[f].flow_id,channel=self.channel,bitrate=self.Queue[f].bitrate,prob_rec_BS1=self.Queue[f].prob_rec_BS1,type=self.Queue[f].type,retransmitted=True,retrans_no=count_Normal) 
                        count_Normal=Retp1.noOfRetransmission()
                        self.deQueue(self.Queue[f].seq_no)
                        self.enQueue(Retp1)
                        BS1_sink=PacketSink(self.env,1, rec_arrivals=True, absolute_arrivals=True, rec_waits=True,debug=True) 
                        BS1_sink.put(Retp1)
                        NoOfRetransmissionNormal+=1
                        print "No of Retransmission Normal%d"%NoOfRetransmissionNormal
						
				######################## Re Transmission for ALARM packets ####################################################################################################		
				#count_Alarm < 3 means we are taking a maximum of 3 retransmission for normal packet
                elif(self.type=="Alarm" and count_Alarm<3  and self.Queue[f].ACKed==False):
                        #BackOff=random.randint(0,self.ContentionWindow)*self.Queue[f].size*8/250
                        BackOff=random.randint(0,math.pow(self.ContentionWindow,count_Normal+1))*self.Queue[f].size*8/250 
                        temp3=self.env.now+BackOff-self.getDeparture(self.Queue[f].seq_no)
                        TotalTime=TotalTime+temp3
                        Retp1 = Packet(self.env.now+BackOff, self.Queue[f].size, count_Alarm+1, src=self.id,seq_no=self.Queue[f].seq_no, flow_id=self.Queue[f].flow_id,channel=self.channel,bitrate=self.Queue[f].bitrate,prob_rec_BS1=self.Queue[f].prob_rec_BS1,type=self.Queue[f].type,retransmitted=True,retrans_no=count_Alarm) 
                        count_Alarm=Retp1.noOfRetransmission()
                        self.deQueue(self.Queue[f].seq_no)
                        self.enQueue(Retp1)
                        BS1_sink=PacketSink(self.env,1, rec_arrivals=True, absolute_arrivals=True, rec_waits=True,debug=True) 
                        BS1_sink.put(Retp1)
                        NoOfRetransmissionAlarm+=1
                        print "No of Retransmission Alarm %d"%NoOfRetransmissionAlarm
						
				#if ACK not received ... deque the packet from the queue and reset the control variables		
                else:
                        self.deQueue(self.Queue[f].seq_no)
                        count_keepAlive=0
                        count_Normal=0
                        count_Alarm=0 
                        
    
	#display the content of the Queue, used for debugging
    def display(self):
        print "queue at time %d in SM %d : %s"%(self.env.now,self.id,self.Queue)                
    
	#the function to fill the queue whenever a new packet is transmitted...or if an old packet is retranmsitted 	
    def enQueue(self,pkt):
        #print "in enQueue "
        self.Queue.append(pkt)
        #print"Queue %s at time %d"%(self.Queue,self.env.now)
        while(len(self.Queue)>self.Queue_Size):
            min_time=self.env.now
            for i in len(self.Queue):
                if(self.Queue[i].time<min_time):
                    min_time=self.Queue[i].time
                    x=i
            self.deQueue(self.Queue[x].seq_no)
			
    #this function returns the oldest packet in the queue      
    def giveOldestPacket(self):
        min_time=self.env.now
        for i in len(self.Queue):
            if(self.Queue[i].time<min_time):
                min_time=self.Queue[i].time
                x=i 
        return self.Queue[x]              
     
	#this function is used to remove the packet from the queue when a packet gets an ACK response or if does not get any ACK after retransmission too
    def deQueue(self,sq):
        #print "in deQueue for %d whose length is %d and at time: %d"%(self.id,len(self.Queue),self.env.now)
        #print "Dequeue from queue %s element with seq no %d"%(self.Queue,sq)
        x=self.getIndex(self.Queue,sq)
        #print "%s"%x
        self.Queue.pop(x)
        #print "After deQueue %s"%self.Queue
    
	#this function return the index of the packet there in the queue
    def getIndex(self,q,sq):
        #print "queue length: %d and we search fr seq no: %d" % ((len(q)),sq)
        for i in range(len(q)):
            #print "queue seq no: %d"%(q[i].seq_no)
            if (q[i].seq_no==sq):
                return i
        return 0   
                
    #get the packet from the queue... this is currently not used but can be used later as per the logic developed in the code  
    def getQueue(self,sq):
        #print "in getQueue "
        #min_departure_time = time
        for i in range(len(self.Queue)):
            if(self.Queue[i].seq_no<=sq):
                return self.Queue[i]
				
    #this can be used when we implement a logic for dynamic contention window based on collision    
    def calcBackOffTime(self,cw):
        BackoffTime=random.randint(1,cw)
        return BackoffTime 
    
    #returns the time at which the packet is sent by the Smart meter     
    def getDeparture(self,sq):
        for i in range(len(self.Queue)):
            
            if(self.Queue[i].seq_no==sq):
                #print "found seq no"
				return self.Queue[i].time
                
        return self.env.now
                
######################################## PacketSink Class ##############################################################################################################################            

class PacketSink(object):
    """ Receives packets and collects delay information into the
        waits list. You can then use this list to look at delay statistics.

        Parameters
        ----------
        env : simpy.Environment
            the simulation environment
        debug : boolean
            if true then the contents of each packet will be printed as it is received.
        rec_arrivals : boolean
            if true then arrivals will be recorded
        absolute_arrivals : boolean
            if true absolute arrival times will be recorded, otherwise the time between consecutive arrivals
            is recorded.
        rec_waits : boolean
            if true waiting time experienced by each packet is recorded
        selector: a function that takes a packet and returns a boolean
            used for selective statistics. Default none.
		check_collision: for checking collision
		

    """
    def __init__(self, env,id, rec_arrivals=False, absolute_arrivals=False, rec_waits=True, debug=False, selector=None):
        self.id=id
        self.store = simpy.Store(env)
        self.env = env
        self.rec_waits = rec_waits
        self.rec_arrivals = rec_arrivals
        self.absolute_arrivals = absolute_arrivals
        #self.check_collision = []
        self.waits = []
        self.arrivals = []
        self.BS_List = []
        self.channel_set = []
        self.pktendtime=[]
        self.prob_rec_BS1=[]
        #self.prob_rec_BS2=[]
        self.debug = debug
        self.action = env.process(self.run())  # starts the run() method as a SimPy process
        self.packets_rec = 0
        self.bytes_rec = 0
        self.selector = selector
        
        #self.packetRecvStatus = True # by default it is true based on which the receiver generates ACK, 
                                      # but when collision it is false and receiver does not generate ACK
        #self.type="keepAlive"
        self.backLogPackets=0
    def run(self):
        last_arrival = 0.0
        
        while True:
            global print_control
            msg = (yield self.store.get())
            self.channel_set.append(msg.channel)
            self.prob_rec_BS1.append(msg.prob_rec_BS1)
            #self.prob_rec_BS2.append(msg.prob_rec_BS2)
            if not self.selector or self.selector(msg):
                now = self.env.now
                if self.rec_waits:
                    self.waits.append(self.env.now - msg.time)
                if self.rec_arrivals:
                    if self.absolute_arrivals:
                        self.arrivals.append(now)
                    else:
                        self.arrivals.append(now - last_arrival)
                    last_arrival = now
                self.packets_rec += 1
                self.bytes_rec += msg.size
                #####to display the output __repr__ of class Packet##################    
                if self.debug:
                    #if((msg.type=="Normal" or msg.type=="Alarm") and msg.src>2):
                            print msg
                    
            #global STATUSA,STATUSB
            #global STATUSA  
            global CollisionCount,collision_check
            #global BS1_ACK_send_STATUS  #if already base station 1 has send the ACK Base station 2 should not send...this Flag is used
            #self.check_collision.append("False")
            #global LoopBackPeriod
            collision_check=0
            global SEQUENCE,ACK_ID,ACK_STATUS,TYPE,N,ACK_PACKET_ID
            downLinkRate=600
            AckPacketSize=14
            #print "print index check : %s and SINK id:%s"%(self.packets_rec-1,self.id)
            if(self.id == 1 ):#if packet Sink is a base station we send here the ACKs based on collision free reception
                #appending the packet end time to the pktendtime list
                self.pktendtime.append(msg.size*8/msg.bitrate + self.arrivals[self.packets_rec-1])
                #collision detection logic...
                #if more than 1 packet received at Base station(for comparison) and channel used by prev and current packet is same 
                #and pkt end time of previous packet is larger than the arrival time of the current packet
                #Then there is collision
                #currently in my logic i am not sending ACK only for the current colliding packet and not for the previous colliding packet
                #Because everytime there is no collision i am sending ACK packets
                #Though i can evaluate the number of collision to be 2 times of the current collision count..
                #but number of retransmission will be half..which is erroneous...i need a little algorithmic help here..
				#if the previous and its previous packets overlaps with current packet
                if (self.packets_rec>10):
                    for i in range(1,10):
                        if (self.channel_set[self.packets_rec-(i+1)]==self.channel_set[self.packets_rec-1]):
                             if((self.pktendtime[self.packets_rec-(i+1)]>self.arrivals[self.packets_rec-1]) and
                                (self.prob_rec_BS1[self.packets_rec-(i+1)]>=pthres and self.prob_rec_BS1[self.packets_rec-1]>=pthres)):
                                #print "Collision Detected at Base Station %s due to same channel %s and packet overlap with previous"%(self.id,self.channel_set[self.packets_rec-1])
                                #self.check_collision=[word.replace("False","True") for word in self.check_collision] 
                                #print "SM ID:%s Packet Id:%s  channel used: %s packets received: %s arrival time: %s  packet end time :%s collision detected :%s "%(msg.src,msg.id,self.channel_set[self.packets_rec-1],self.packets_rec,self.arrivals[self.packets_rec-1],self.pktendtime[self.packets_rec-1],self.check_collision[self.packets_rec-1])
                                #print " No ACK transmitted due to collision"
                                collision_check=1
                                if(msg.type=="Normal" or msg.type=="Alarm"):
                                    print i
                                    CollisionCount+=1
                                    print"collision count=%d"%CollisionCount
                                break
                    if(self.prob_rec_BS1[self.packets_rec-1]>=pthres and collision_check!=1):
                        sendingACKtime=now+random.randint(4,6)+(msg.size*8/msg.bitrate)
                        #print "BS ID:%s sending ACK to SM ID:%s AT TIME: %0.4f"%(1,msg.src,sendingACKtime)
                        SinkObject=PacketSink(self.env,msg.src, rec_arrivals=True, absolute_arrivals=True, rec_waits=True,debug=True) 
                        ACK=PacketGenerator(self.env, 1,sendingACKtime-now,AckPacketSize, msg.channel ,downLinkRate,1,msg.seq_no+1,msg.type)
                        ACK.out=SinkObject
                         
                elif(self.prob_rec_BS1[self.packets_rec-1]>=pthres):    
                    sendingACKtime=now+random.randint(4,6)+(msg.size*8/msg.bitrate)
                    #print "BS ID:%s sending ACK to SM ID:%s AT TIME: %0.4f"%(1,msg.src,sendingACKtime)
                    SinkObject=PacketSink(self.env,msg.src, rec_arrivals=True, absolute_arrivals=True, rec_waits=True,debug=True) 
                    ACK=PacketGenerator(self.env, 1,sendingACKtime-now,AckPacketSize, msg.channel ,downLinkRate,1,msg.seq_no+1,msg.type)
                    ACK.out=SinkObject
                
                    
                    
					
            #if packet sink is a Smart meter....we check the ACK reception status
            else:
                    
                #print "ACK received at SM %s"%self.id
                for i in range(0,N):
                    #print "inside for loop %d and compared index %d"%(self.id,i+3)
                    if(self.id==i+3):
                        #print "inside if loop"
                        ACK_STATUS[i]=True
                        ACK_ID[i]=self.id
                        #ACK_PACKET_ID=msg.id
                        SEQUENCE=msg.seq_no
                        TYPE[i]=msg.type
                        #ACK_TIME[i]=self.env.now
                        #print msg.type
               
    def put(self, pkt):
        self.store.put(pkt)