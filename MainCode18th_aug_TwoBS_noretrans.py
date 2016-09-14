import numpy
import math
import random
import simpy

from SimLibraryTwoBS_aug18_noretrans import *
import scipy.stats as stats
#from SimLibraryTwoBS import CollisionCount

############################################### Arrival Traffic Function Definitions############################################# 
#control variables
k=1
j=1
i=1
cache=0
cache1=0
cache2=0
cache3=0
cache4=0
reportInterval_SM = 60 #60 sec time for keep alive
reportInterval_SM_Normal = 300 #5 mins
Mean_Report_Interval = 3600 # 1 hour for Aperiodic packets   
N=300#no of Smart meters
itrn=1
itrn2=1
#LoopbackTime=12
#for uplink packet from smart meter to BS
SIMULATION=2000
def ArrivalTimeSM_keepAlive():
    #need to set a random start time - any second within one hour - 0 to 3600 sec
    global cache,k
    #global reportInterval_SM,reportInterval_SM_Normal
    reportInterval_SM = 60 #60 sec time for keep alive
    a = 0
    b = reportInterval_SM - 1
    #print "k %s"%k
    if(k<=2*N):#as we are using one Base station it is N...for 2 Base station it will be 2*N...for 3 it will be 3*N..for initial offset
        if k%2==1 :
            k=k+1 
            #cache=random.randint(a,b)
            cache=random.uniform(a,b)
            return cache
        else:
            k=k+1
            return cache
    else:#after all the smart meter have an initial offset the 60 sec interval follows 
        k=k+1
        cache = reportInterval_SM #then the arrival rate of 60 follows
        return cache
        

#####Defining Arrival Rate for Normal packets#########              
def ArrivalTimeSM_Normal():
    #need to set a random start time - any second within one hour - 0 to 3600 sec
    global cache1,j
    reportInterval_SM_Normal = 300#5 min time for Normal packets
    a = 0
    b = reportInterval_SM_Normal - 1
    #print "k %s"%k
    if(j<=2*N):#as we are using one Base station it is N...for 2 Base station it will be 2*N...for 3 it will be 3*N..for initial offset
        if j%2==1:
            j=j+1 
            #cache1=random.randint(a,b)
            cache1=random.uniform(a,b)
            return cache1
        else:
            j=j+1
            return cache1
        
    else:#the second packet sent to another Base station is the replica of the first one
        j=j+1
        cache1 = reportInterval_SM_Normal#then the arrival rate of 14400 follows
        return cache1    
        
#####Defining Arrival Rate for Alarm packets#########
def ArrivalTimeSM_Alarm(): 
    global Mean_Report_Interval ,i ,cache2
    if( i%2==1): 
        i=i+1      
        cache2=math.ceil(random.expovariate(1.0/Mean_Report_Interval)) #exponential arrival rate with a mean of one hour
        return cache2
    else:
        i=i+1
        return cache2
    
                                    
#########Defining keepAlive,Normal and Alarm packets for Household meters################################              
def upLinkUser_HouseHoldPkt(str): # Constant packet size distribution 90 percent of users
    if(str=="keepAlive"):
    #92 bytes the packet size+ 132 bits * (92/32) 
    #headers :40 bit MAC header, 12 bit Seq Counter, 40 bit node identifier, 16 bit Auth Hash, 8 bits CRC, 16 bits ECC
        return 141.5 #constant payload of 92 bytes + 3*16.5 bytes(132 bits)  as 92 byte/32 bytes= 3 
    elif(str=="Normal" ):
    #254 bytes the packet size+ 132 bits *(254/32) = 254 + 8*16.5 bytes
        return 386
    elif(str =="Alarm"):
    #taking an exponential distribution for the size of the packet with mean 200 as per the discusiion with Proff
        global itrn,cache3
        mean=200
        if(itrn%2==1):
            itrn+=1
            size=random.expovariate(1.0/mean)
            cache3= math.ceil(size+math.ceil(size/32)*16.5)
            return cache3
        else:
            itrn+=1
            return cache3
        
#########Defining keepAlive,Normal and Alarm packets for Commercial meters################################        
def upLinkUser_CommercialPkt(str): 
    if(str=="keepAlive"):
    #double of Household
        return 2*141.5 
    elif(str=="Normal"):  
    #double of household
        return 2*386   
    elif(str =="Alarm"):
     #taking an exponential distribution for the size of the packet with mean 400 as per the discusiion with Proff
        
        global itrn2,cache4
        mean2=400
        if(itrn2%2==1):
            itrn2+=1
            size=random.expovariate(1.0/mean2)
            cache4= math.ceil(size+math.ceil(size/32)*16.5)
            return cache4
        else:
            itrn2+=1
            return cache4
        
       
##################Start of Simulation###########################################       
start = True
env = simpy.Environment()  # Create the SimPy environment    
#log normal calculation, currently i havent utilized these in the consideration of collision
d0 = 1             #reference distance = 1 meter
prd0 = 0           #received signal at reference distance is 0 dBm
n = 3.5            #path loss coefficient. Assume urban area
prd_req = -120     #minimum required received power at base station 
sigma = 8          # standard deviation of 8 dB
upLinkBitRate = 250
downLinkBitRate= 600
household=0
commercial=0

#ContentionWindow = 4 #for BackOff calculation
BS1_id=1           #Base Station 1 ID
BS2_id=2

#Base Station 1 and 2 SINK instantiated
BS1_PS = PacketSink(env,BS1_id, rec_arrivals=True, absolute_arrivals=True, rec_waits=True, debug=True, selector=None)  # debugging enable for simple output
BS2_PS = PacketSink(env,BS2_id, rec_arrivals=True, absolute_arrivals=True, rec_waits=True, debug=True, selector=None)  # debugging enable for simple output


#Smart Meter ID would range from 3 to 102 (I am considering 100 Smart Meters)
SM_id=3
#creates a 100 x 2 matrix to store the Id 
#Channel ID would range from 1 to 8
channel_size=8

#sequence number would be used in further simulations..currently kept inactive
#max_seq_Num=4095
#seq_Num=0
#seq_Num=1

#base station co-ordinate
BS1_Loc = numpy.array((1154,1000))
BS2_Loc = numpy.array((2885,1000))

BS1_List =[]   
BS2_List=[]
#carrying out the experiment for 100 smart meters
while(SM_id<3+N):
        
        new=[]
        new.append(SM_id)
        a = random.normalvariate(5,0.217)
        #assuming a square cell with a diagonal of 2000m
        x_cord = random.randint(0,3750) #x coordinate of the smart meter node
        #x_cord = random.randint(0,2000) #x coordinate of the smart meter node
        y_cord = random.randint(0,2000) #y coordinate of the smart meter node
        cord = numpy.array((x_cord,y_cord))#cord holds the coordinate of the smart meter
        distBS1 = numpy.linalg.norm(cord-BS1_Loc)# Distance between the Smart meter and the Base Station
        distBS2 = numpy.linalg.norm(cord-BS2_Loc)
        
        prd_mean_BS1=prd0 -10*n*math.log10(distBS1/d0)#power received at Base Station from the smart meters
        prd_mean_BS2=prd0 -10*n*math.log10(distBS2/d0)
        #calculation of Q value...probability that the received signal is higher than -120dB
        prob_rec_BS1=0.5*math.erfc((prd_req-prd_mean_BS1)/sigma*math.sqrt(2))
        prob_rec_BS2=0.5*math.erfc((prd_req-prd_mean_BS2)/sigma*math.sqrt(2))
        
        if a<5.278:#for 0.9 probability for household Smart meters
            
            
            #Keep Alive Packet from the smart meter
            #SM_PG3 = PacketGenerator(env, SM_id,ArrivalTimeSM_keepAlive,upLinkUser_HouseHoldPkt("keepAlive"),channel_size,upLinkBitRate,prob_rec_BS1,0,"keepAlive")
            SM_PG3 = PacketGenerator(env, SM_id,ArrivalTimeSM_keepAlive,upLinkUser_HouseHoldPkt("keepAlive"),channel_size,upLinkBitRate,prob_rec_BS1,prob_rec_BS2,0,"keepAlive")
            SM_PG4 = PacketGenerator(env, SM_id,ArrivalTimeSM_keepAlive,upLinkUser_HouseHoldPkt("keepAlive"),channel_size,upLinkBitRate, prob_rec_BS1,prob_rec_BS2,0,"keepAlive")
            #Normal data packet from the smart meter
            #SM_PG1 = PacketGenerator(env, SM_id,ArrivalTimeSM_Normal, upLinkUser_HouseHoldPkt("Normal"), channel_size,upLinkBitRate, prob_rec_BS1,0,"Normal")
            SM_PG1 = PacketGenerator(env, SM_id,ArrivalTimeSM_Normal, upLinkUser_HouseHoldPkt("Normal"), channel_size,upLinkBitRate, prob_rec_BS1,prob_rec_BS2,0,"Normal")
            SM_PG2 = PacketGenerator(env, SM_id,ArrivalTimeSM_Normal, upLinkUser_HouseHoldPkt("Normal"), channel_size,upLinkBitRate, prob_rec_BS1,prob_rec_BS2,0,"Normal") 
            #Alarm packet from the smart meter
            #SM_PG5 = PacketGenerator(env, SM_id,ArrivalTimeSM_Alarm, upLinkUser_HouseHoldPkt("Alarm"), channel_size,upLinkBitRate,prob_rec_BS1,0,"Alarm")
            SM_PG5 = PacketGenerator(env, SM_id,ArrivalTimeSM_Alarm, upLinkUser_HouseHoldPkt("Alarm"), channel_size,upLinkBitRate,prob_rec_BS1,prob_rec_BS2,0,"Alarm")
            SM_PG6 = PacketGenerator(env, SM_id,ArrivalTimeSM_Alarm, upLinkUser_HouseHoldPkt("Alarm"), channel_size,upLinkBitRate, prob_rec_BS1,prob_rec_BS2,0,"Alarm")
            #as per the mail from Dr goulart the upLinkUserPkt() will have exponential distribution..poisson traffic.currently considering constant for single user
            household=household+1
        #array to hold the SM id and channel used
        else:
        
            
            #Keep Alive Packet from the smart meter
            #SM_PG3 = PacketGenerator(env, SM_id,ArrivalTimeSM_keepAlive, upLinkUser_CommercialPkt("keepAlive"), channel_size,upLinkBitRate,prob_rec_BS1,0,"keepAlive")
            SM_PG3 = PacketGenerator(env, SM_id,ArrivalTimeSM_keepAlive, upLinkUser_CommercialPkt("keepAlive"), channel_size,upLinkBitRate,prob_rec_BS1,prob_rec_BS2,0,"keepAlive")
            SM_PG4 = PacketGenerator(env, SM_id,ArrivalTimeSM_keepAlive, upLinkUser_CommercialPkt("keepAlive"),channel_size,upLinkBitRate,prob_rec_BS1,prob_rec_BS2,0,"keepAlive")
            #Normal data packet from the smart meter
            #SM_PG1 = PacketGenerator(env, SM_id,ArrivalTimeSM_Normal, upLinkUser_CommercialPkt("Normal"),channel_size,upLinkBitRate,prob_rec_BS1,0,"Normal")
            SM_PG1 = PacketGenerator(env, SM_id,ArrivalTimeSM_Normal, upLinkUser_CommercialPkt("Normal"),channel_size,upLinkBitRate,prob_rec_BS1,prob_rec_BS2,0,"Normal")
            SM_PG2 = PacketGenerator(env, SM_id,ArrivalTimeSM_Normal, upLinkUser_CommercialPkt("Normal"), channel_size,upLinkBitRate,prob_rec_BS1,prob_rec_BS2,0,"Normal") 
            #Alarm packet from the smart meter
            #SM_PG5 = PacketGenerator(env, SM_id,ArrivalTimeSM_Alarm, upLinkUser_CommercialPkt("Alarm"), channel_size,upLinkBitRate,prob_rec_BS1,0,"Alarm")
            SM_PG5 = PacketGenerator(env, SM_id,ArrivalTimeSM_Alarm, upLinkUser_CommercialPkt("Alarm"), channel_size,upLinkBitRate,prob_rec_BS1,prob_rec_BS2,0,"Alarm")
            SM_PG6 = PacketGenerator(env, SM_id,ArrivalTimeSM_Alarm, upLinkUser_CommercialPkt("Alarm"),channel_size,upLinkBitRate,prob_rec_BS1,prob_rec_BS2,0,"Alarm")
            #as per the mail from Dr goulart the upLinkUserPkt() will have exponential distribution..poisson traffic.currently considering constant for single user
            commercial=commercial+1

        SM_PG2.out = BS2_PS
        SM_PG1.out = BS1_PS
        
        SM_PG4.out = BS2_PS
        SM_PG3.out = BS1_PS
        
        SM_PG6.out = BS2_PS
        SM_PG5.out = BS1_PS
        
            
        #print"SM_id is %s "%SM_id
 
        SM_id=SM_id+1  
env.run(until=SIMULATION) #until how much time we run the simulation
#global CollisionCount,NoOfRetransmission
#print "Total Packets received by the Base station is %s"%BS1_PS.packets_rec
#print "number of collisions: %d"%SimLibraryTwoBS.CollisionCount
