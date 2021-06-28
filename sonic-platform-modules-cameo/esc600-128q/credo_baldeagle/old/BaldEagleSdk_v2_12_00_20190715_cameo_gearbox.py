####################################################################################################
# 2019-07-03, Credo BALD EAGLE SDK Software scripts
#
# CREDO Semiconductors Inc. Confidential
# 
####################################################################################################
# These sets of scripts are for use with Bald Eagle, with the Binary FW
#
# To be able to run the functions inside this file, execute this file at the python prompt:
#
#      >>> execfile("CredoSdk.py")
# 
#
####################################################################################################
from __future__ import division
import re, sys, os, time, datetime, struct
from sys import *
#import numpy as np
import math

import random
from ctypes import *
sys.path.insert(0,os.getcwd())
cameolibpath=os.getenv('CAMEOLIB')
fw_path=os.getenv('CREDO_100G_PATH')
#libcameo = ctypes.CDLL("libcameo_mdio.so")
cdll.LoadLibrary(cameolibpath)
libcameo = CDLL(cameolibpath)
import threading
lock1 = threading.Lock()
lock2 = threading.Lock()
MdioRdLock = threading.Lock()
MdioWrLock = threading.Lock()
#import /usr/lib/python2.7/site-packages/DosComponent/BabbageLib/baldeagle_phoenix_reg as phoenix
#import /usr/lib/python2.7/site-packages/DosComponent/BabbageLib/baldeagle_eagle_reg as eagle

import baldeagle_phoenix_reg as Pam4Reg
import baldeagle_eagle_reg as NrzReg
#try:
#    reload(NrzReg)
#    reload(Pam4Reg)
#except:
#    pass

##########################[   0       1       2       3      4      5      6       7       8       9     10      11     12      13     14      15 ] 
lane_name_list =          [ 'A0',   'A1',   'A2',   'A3',  'A4',  'A5',   'A6',   'A7',  'B0',   'B1',  'B2',   'B3',  'B4',   'B5',  'B6',   'B7']
#lane_rx_input_mode_list = [ 'dc',   'dc',   'dc',
try:
    lane_rx_input_mode_list
except:
    lane_rx_input_mode_list = ['dc']*16
try:
    lane_mode_list
except:
    lane_mode_list = ['pam4'] *16
try:
    chan_est_list
except:
    chan_est_list = [1.010,63,62] *16

    
lane_offset = { 'A0': 0x7000,'A1': 0x7200,'A2': 0x7400,'A3': 0x7600,
                'A4': 0x7800,'A5': 0x7a00,'A6': 0x7c00,'A7': 0x7e00,
                'B0': 0x8000,'B1': 0x8200,'B2': 0x8400,'B3': 0x8600,
                'B4': 0x8800,'B5': 0x8a00,'B6': 0x8c00,'B7': 0x8e00,}

gPrbsPrevCount  = { 'A0': 0,'A1': 0,'A2': 0,'A3': 0,
                    'A4': 0,'A5': 0,'A6': 0,'A7': 0,
                    'B0': 0,'B1': 0,'B2': 0,'B3': 0,
                    'B4': 0,'B5': 0,'B6': 0,'B7': 0,}
                    
    
MDIO_CONNECTED       = 0
MDIO_DISCONNECTED    = 1
MDIO_LOST_CONNECTION = 2
global gChipName;             gChipName='Baldeagle'
global gSetupFileName;        gSetupFileName='Baldeagle_RegSetup.txt'
global gUsbPort;              gUsbPort=0 # eval board USB port0 or port 1, Can run 2 Credo boards in 2 python shells on the same PC
global gSlice;                gSlice=0 # Slice 0 or 1
global gRefClkFreq;           gRefClkFreq=195.3125  # 156.25
global gNrzLanes;             gNrzLanes = [8,9,10,11,12,13,14,15] #[0,1,2,3,4,5,6,7]
global gPam4Lanes;            gPam4Lanes= [0,1,2,3,4,5,6,7] #[8,9,10,11,12,13,14,15]
global gPrbsEn;               gPrbsEn=True
global gLane;                 gLane=range(16) #[0,1,2,3,4,5,6,7] #or 'ALL'
global gDebugPrint;           gDebugPrint=1
global gNrzTxSourceIsCredo;   gNrzTxSourceIsCredo  =1 # 0: means do not touch  NRZ lane's TX taps, as it's driving another NRZ lane's RX
global gPam4TxSourceIsCredo;  gPam4TxSourceIsCredo =1 # 0: means do not touch PAM4 lane's TX taps, as it's driving another NRZ lane's RX
global gFecThresh;            gFecThresh=15
global gSltVer;               gSltVer=0.0
global gDualSliceMode;        gDualSliceMode=False
global c; c=Pam4Reg
global RxPolarityMap
global TxPolarityMap
global RxGrayCodeMap
global TxGrayCodeMap
global RxPrecoderMap
global TxPrecoderMap
global RxMsbLsbMap;   
global TxMsbLsbMap
global LanePartnerMap
global gFwFileName
global gFwFileNameLastLoaded



gPrbsResetTime=time.time()
gDebugTuning = False
try:
    gDebugTuning
except:
    gDebugTuning = False
try:
    gPrint
except:
    gPrint = True
try:
    gLane
except:
    gLane = [0]


##########################
# Define Globals here once
########################## SMK Board's TX Source for each lane's RX is its own TX (assuming loop-back)
global LanePartnerMap;
LanePartnerMap=[[[0,0],[0,1],[0,2],[0,3],[0,4],[0,5],[0,6],[0,7],  [0,8],[0,9],[0,10],[0,11],[0,12],[0,13],[0,14],[0,15]], # Slice 0, TX Source for each lane's RX
           [[1,0],[1,1],[1,2],[1,3],[1,4],[1,5],[1,6],[1,7],  [1,8],[1,9],[1,10],[1,11],[1,12],[1,13],[1,14],[1,15]]] # Slice 1, TX Source for each lane's RX
########################## SMK Board's RX and TX Polarities for Slice 0 and 1
global RxPolarityMap; RxPolarityMap=[]
global TxPolarityMap; TxPolarityMap=[]
                              #Slice  [A0.............A7, B0,..........B7]
RxPolarityMap.append([]); RxPolarityMap[0]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 0 lanes, RX Polarity
TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 1 lanes, RX Polarity
TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
########################## Channel Estimates per lane per Slice
global gChanEst; gChanEst=[]    
                              # [Chan Est, OF, HF]
gChanEst.append([]); gChanEst[0]=[[0.0,0,0]]*16 # Slice 0, Channel Estimates for each lane's RX
gChanEst.append([]); gChanEst[1]=[[0.0,0,0]]*16 # Slice 1, Channel Estimates for each lane's RX
########################## PRBS and BER Statistics per lane per Slice
global gLaneStats; gLaneStats=[]    
                 #[PrbsCount, PrbsCount-1, PrbsCount-2, PrbsRstTime, PrbsLastReadoutTime]
gLaneStats.append([]); gLaneStats[0]=[[0,0,0,0,0,0,0,0,0,0,0]]*16 # Slice 0, PRBS Statistics for each lane's RX
gLaneStats.append([]); gLaneStats[1]=[[0,0,0,0,0,0,0,0,0,0,0]]*16 # Slice 1, PRBS Statistics for each lane's RX

########################## Line Encoding mode and Data Rate per lane per Slice
global gEncodingMode; gEncodingMode=[]    
                 #[PAM4/NRZ, 53.125/25.78125/20.625/10.3125]
gEncodingMode.append([]); gEncodingMode[0]=[['pam4', 53.125]]*16 # Slice 0, Data Rate for each lane, will be updated by pll() or init_pam4/init_nrz
gEncodingMode.append([]); gEncodingMode[1]=[['pam4', 53.125]]*16 # Slice 1, Data Rate for each lane, will be updated by pll() or init_pam4/init_nrz

####################################################################################################
def slice_power_up_init(slice=0):
    
    slices = get_slice_list(slice)
    need_to_load_fw = False
    
    for slc in slices:
        print("\n...Slice %d Power-Up-Initialization Started..."%slc),
        sel_slice(slc)
        slice_reset()
        set_bandgap('on', range(16)) 
        set_top_pll(pll_side='both', freq=195.3125)
        pll_cal_top()
        if not fw_loaded():
            need_to_load_fw = True
    if need_to_load_fw:
        fw_load(sl=slices)

    return slices
    
####################################################################################################
def config_baldeagle(slice=0, mode='nrz', input_mode='ac', lane=None, cross_mode=False):
    
    global RxPolarityMap; 
    if 'RETIMER' in mode.upper():
        print("\n*** RETIMER mode selected.\n")
        lanes = get_retimer_lane_list(lane, cross_mode)
    else:
        lanes = get_lane_list(lane) 
    global gLane; gLane=lanes
    slices = slice_power_up_init(slice)

    for slc in slices:
        sel_slice(slc)
                                      #   Lane No: [A0.A1.A2.A3.A4.A5.A6.A7,B0.B1.B2.B3.B4.B5.B6.B7 ]
        RxPolarityMap.append([]); RxPolarityMap[0]=[ 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1 ] # Slice 0 lanes, RX Polarity
        RxPolarityMap.append([]); RxPolarityMap[0]=[ 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0 ] # Slice 0 lanes, RX Polarity
        RxPolarityMap.append([]); RxPolarityMap[1]=[ 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1 ] # Slice 1 lanes, RX Polarity
        init_lane (mode=mode,input_mode=input_mode,lane=lanes) 

    for slc in slices:
        sel_slice(slc)
        #fw_reg(115,0xffff)  # Disable FFE Adaptation
        #reg(0x20,0x380)     # CDR BW Kp=6
        #reg(0x1d6,0xcc80)   # az short pulse
        #reg(0x7b,0x0004)    # Enable BLWC
        if 'RETIMER' in mode.upper(): # if Retimer Mode
            prbs_mode_select(lane=lanes, prbs_mode='functional')
            fw_config_retimer (mode=mode, lane=lanes, cross_mode=cross_mode)
            fw_config_wait (max_wait=None, lane=lanes, print_en=0)
            #reg(0xA0,0x320,lane=lanes) # enable TX output back to fucntional. FW disabled it.
        else: # Phy mode or OFF mode  
            fw_config_lane (mode=mode,lane=lanes)
            
    if 'OFF' not in mode.upper():
        for slc in slices:
            sel_slice(slc)
            fw_adapt_wait (max_wait=None, lane=lanes, print_en=1)
            fw_config_wait(max_wait=None, lane=lanes, print_en=1)
        
        for slc in slices:
            sel_slice(slc)
            fw_serdes_params(lane=lanes)

        for slc in slices:
            sel_slice(slc)
            if 'RETIMER' in mode.upper():
                auto_pol(port='tx',tx_prbs='dis') # Do not enable TX PRBS Gen if in RETIMER mode, for auto polarity correction
                auto_pol(port='rx',tx_prbs='dis') # Do not enable TX PRBS Gen if in RETIMER mode, for auto polarity correction
            else:            
                auto_pol(tx_prbs='en') # Enable TX PRBS Gen if in SERDES mode, for auto polarity correction
                
    # for slc in slices:
        # sel_slice(slc)
        # fw_slice_params(lane='ALL') 

    for slc in slices:
        sel_slice(slc)
        if gSltVer>0: slt_mode(slt_ver=gSltVer)        
    for slc in slices:
        sel_slice(slc)
        rx_monitor(rst=1,lane=lanes)
  
    sel_slice(slices[0])
####################################################################################################
def config_active_switchover_orig(mode='direct'):

    cross_mode = False if 'dir' in mode else True
    global TxPolarityMap; 
    global RxPolarityMap;
    slice=0
    mode='nrz'
    A_lanes=[0,1,2,3]
    B_lanes0=[8,9,10,11]
    B_lanes1=[12,13,14,15]
    active_B_lanes    = B_lanes0
    nonactive_B_lanes = B_lanes1
    
    if cross_mode==True:
        active_B_lanes    = B_lanes1
        nonactive_B_lanes = B_lanes0
        
    all_lanes=A_lanes+B_lanes0+B_lanes1
    
    lanes = get_retimer_lane_list(A_lanes, cross_mode)

    global gLane; gLane=all_lanes
    slices = slice_power_up_init(slice)

    for slc in slices:
        sel_slice(slc)
                                      #   Lane No: [A0.A1.A2.A3.A4.A5.A6.A7,B0.B1.B2.B3.B4.B5.B6.B7 ]
        TxPolarityMap.append([]); TxPolarityMap[0]=[ 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0 ] # Slice 0 lanes, TX Polarity
        RxPolarityMap.append([]); RxPolarityMap[0]=[ 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0 ] # Slice 0 lanes, RX Polarity
        TxPolarityMap.append([]); TxPolarityMap[1]=[ 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1 ] # Slice 1 lanes, TX Polarity
        RxPolarityMap.append([]); RxPolarityMap[1]=[ 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0 ] # Slice 1 lanes, RX Polarity
        init_lane (mode='nrz',lane=all_lanes) 
        
    for slc in slices:
        sel_slice(slc)
        prbs_mode_select(lane= A_lanes+active_B_lanes, prbs_mode='functional')
        prbs_mode_select(lane= nonactive_B_lanes,      prbs_mode='prbs')
        
        fw_config_retimer (mode='nrz', lane=lanes, cross_mode=cross_mode)
        fw_config_lane (mode='nrz', lane=nonactive_B_lanes) 
        #tx_output(mode='EN',lane=A_lanes+active_B_lanes, print_en=0)   
        for ln in range(16):
            # Set the polarities of the lanes
            pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
        fw_config_wait (max_wait=None, lane=all_lanes, print_en=1)
        
    for slc in slices:
        sel_slice(slc)
        fw_serdes_params(lane=all_lanes)
                
    for slc in slices:
        sel_slice(slc)
        fw_slice_params(lane=all_lanes) 

    for slc in slices:
        sel_slice(slc)       
        #tx_output(mode='EN',lane=A_lanes+active_B_lanes, print_en=0)
        for ln in range(16):
            # Set the polarities of the lanes
            pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
        rx_monitor(rst=1,lane=all_lanes)
  
    sel_slice(slices[0])
####################################################################################################
def config_active_switchover(mode=None):

    slc=0
    global TxPolarityMap; 
    global RxPolarityMap;
                                  #   Lane No: [A0.A1.A2.A3.A4.A5.A6.A7,B0.B1.B2.B3.B4.B5.B6.B7 ]
    TxPolarityMap.append([]); TxPolarityMap[0]=[ 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0 ] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[0]=[ 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0 ] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[ 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1 ] # Slice 1 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[ 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0 ] # Slice 1 lanes, RX Polarity


    A_lanes=[0,1,2,3]
    B_lanes0=[8,9,10,11]
    B_lanes1=[12,13,14,15]
    cross_mode = True if mode!=None and 'cross' in mode else False
    if cross_mode==True:
        main_B_lanes    = B_lanes1
        standby_B_lanes = B_lanes0
    else:
        main_B_lanes    = B_lanes0
        standby_B_lanes = B_lanes1
    
    all_lanes= A_lanes + main_B_lanes + standby_B_lanes
    global gLane; gLane=all_lanes
        
    if mode!=None and 'init' in mode.lower():
        slices = slice_power_up_init(slc)
        init_lane (mode='nrz',lane=all_lanes)
        for ln in range(16): # Set the polarities of the lanes        
            pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
        fw_reg_wr(9,0xffff)        
        fw_reg_wr(8,0xffff)        
        fw_reg_wr(128,0xffff)        
        fw_reg_wr(115,0)        
        prbs_mode_select(lane= all_lanes, prbs_mode='functional')
        prbs_mode_select(lane= standby_B_lanes, prbs_mode='prbs')
        fw_config_retimer (mode ='nrz', lane = A_lanes, cross_mode = cross_mode)
        fw_config_lane    (mode ='nrz', lane = standby_B_lanes)
        fw_adapt_wait  (max_wait=None, lane=all_lanes, print_en=1)                              
        fw_config_wait (max_wait=None, lane=A_lanes,   print_en=1)    
        fw_serdes_params(lane=all_lanes)            
        fw_slice_params(lane=all_lanes) 
        fw_reg_wr(8,0)
        fw_reg_wr(128,0)
    else:
        #fw_config_lane    (mode ='nrz', lane = standby_B_lanes)
        if mode==None and rreg([0x9855,[15,12]])==0xF:  # Already in Crossed mode, switch to Direct mode
            print("\n Swithing from Cross Mode to Direct Mode")
            cross_mode= False
        else:
            print("\n Swithing from Direct Mode to Cross Mode")
            cross_mode= True
        fw_config_retimer (mode ='nrz', lane = A_lanes, cross_mode = cross_mode)
            
        # wreg([0xf4,[0]],0,lane=None)
        # if cross_mode==False:
            # wreg(0x9855,0x0000) # Direct lane mapping for retimer mode
            # wreg(0x985f,0x0000) # Direct AN lane mapping for retimer mode
            # wreg(0x98d1,0x0000) # lanesA7-A4: A7TX<-B7RX, A6TX<-B6RX, A5TX<-B5RX, A4TX<-B4RX            
            # wreg(0x98d2,0x3210) # lanesA3-A0: A3TX<-B3RX, A2TX<-B2RX, A1TX<-B1RX, A0TX<-B0RX
            
            # #wreg(0x98d3,0x7654) # lanesB7-B4: B7TX<-A7RX, B6TX<-A6RX, B5TX<-A5RX, B4TX<-A4RX
            # wreg(0x98d3,0x0000) # lanesB7-B4: B7TX<-A3RX, B6TX<-A2RX, B5TX<-A1RX, B4TX<-A0RX
            # wreg(0x98d4,0x3210) # lanesB3-B0: B3TX<-A3RX, B2TX<-A2RX, B1TX<-A1RX, B0TX<-A0RX
        # else: # CROSSED Mode
            # wreg(0x9855,0xF00F) # Crossed lane mapping for retimer mode
            # #wreg(0x985f,0xFF00) # Crossed AN lane mapping for retimer mode
            # wreg(0x98d1,0x0000) # lanesA7-A4: A7TX<-B3RX, A6TX<-B3RX, A5TX<-B1RX, A4TX<-B0RX           
            # wreg(0x98d2,0x7654) # lanesA3-A0: A3TX<-B7RX, A2TX<-B6RX, A1TX<-B5RX, A0TX<-B4RX
            
            # wreg(0x98d3,0x3210) # lanesB7-B4: B7TX<-A3RX, B6TX<-A2RX, B5TX<-A1RX, B4TX<-A0RX
            # wreg(0x98d4,0x0000) # lanesB3-B0: B3TX<-A3RX, B2TX<-A2RX, B1TX<-A1RX, B0TX<-A0RX
            # #wreg(0x98d4,0x7654) # lanesB3-B0: B3TX<-A7RX, B2TX<-A6RX, B1TX<-A5RX, B0TX<-A4RX
        # wreg([0xf4,[0]],1,lane=None)

    
    reg([0x9855,0x985f,0x98d1,0x98d2,0x98d3,0x98d4])
   

    #tx_output(mode='EN',lane=A_lanes+active_B_lanes, print_en=0)   
    #rx_monitor(rst=1,lane=all_lanes)
  
####################################################################################################
def config_baldeagle_gearbox(slice=0, A_lanes=None, gearbox_type='100G-1', gearbox_by_fw=True, fec_b_bypass=False):
    #fw_load(gFwFileName)

    if '50' in gearbox_type:
        # For 50G-2 Gearbox mode, 3 options supported for A-Lane groups 
        group0_50G=[0] # A_lanes group 1 -> [A0] <-> [ 8, 9]
        group1_50G=[1] # A_lanes group 2 -> [A1] <-> {10,11]
        group2_50G=[2] # A_lanes group 3 -> [A2] <-> [12,13]
        group3_50G=[3] # A_lanes group 4 -> [A3] <-> [14,15]
        
        #Determine the corresponding B-Lanes for each group of A-Lanes
        group0_selected=0
        group1_selected=0
        group2_selected=0
        group3_selected=0

        #Determine the corresponding B-Lanes for each group of A-Lanes
        B_lanes=[]
        if all(elem in A_lanes for elem in group0_50G): # If A_lanes contains [0]
            B_lanes+=[ 8, 9]                        
            group0_selected=1
        if all(elem in A_lanes for elem in group1_50G): # If A_lanes contains [1]
            B_lanes+=[10,11]                      
            group1_selected=1
        if all(elem in A_lanes for elem in group2_50G): # If A_lanes contains [2]
            B_lanes+=[12,13]
            group2_selected=1
        if all(elem in A_lanes for elem in group3_50G): # If A_lanes contains [3]
            B_lanes+=[14,15]
            group3_selected=1
        if group0_selected==0 and group1_selected==0 and group2_selected==0 and group3_selected==0:
            print("\n*** 50G-1 Gearbox Setup: Invalid Target A-Lanes specified!"),
            print("\n*** Options: A_lanes=[0]"),
            print("\n***          A_lanes=[1]"),
            print("\n***          A_lanes=[2]"),
            print("\n***          A_lanes=[3]"),
            print("\n***          A_lanes=[0,1,2,3]")
            return
    else: # '100' in gearbox_type
        # For 100G-2 Gearbox mode, 3 options supported for A-Lane groups 
        group0_100G=[0,1] # A_lanes group 1 -> [A0,A1] <-> [ 8, 9,10,11]
        group1_100G=[2,3] # A_lanes group 2 -> [A2,A3] <-> [12,13,14,15]
        group2_100G=[4,5] # A_lanes group 3 -> [A4,A5] <-> [12,13,14,15]
        
        #Determine the corresponding B-Lanes for each group of A-Lanes
        group0_selected=0
        group1_selected=0
        group2_selected=0
        B_lanes=[]
        if all(elem in A_lanes for elem in group0_100G):  # If A_lanes contains [0,1]
            B_lanes+=[8,9,10,11]
            group0_selected=1
        if all(elem in A_lanes for elem in group1_100G):  # If A_lanes contains [2,3]
            B_lanes+=[12,13,14,15]
            group1_selected=1
        elif all(elem in A_lanes for elem in group2_100G): # If A_lanes contains [4,5]
            B_lanes+=[12,13,14,15]
            group2_selected=1
        if group1_selected==0 and group2_selected==0: # can only select group 1 or 2, not both
            print("\n*** 100G-1 Gearbox Setup: Invalid Target A-Lanes specified!"),
            print("\n*** Options: A_lanes=[0,1]"),
            print("\n***          A_lanes=[2,3]"),
            print("\n***          A_lanes=[4,5]"),
            print("\n***          A_lanes=[0,1,2,3]"),
            print("\n***          A_lanes=[0,1,4,5]")
            return
        
    lanes = get_lane_list(lane=A_lanes+B_lanes)
    global gLane; gLane=lanes    
    slices = slice_power_up_init(slice)
    
    if gearbox_by_fw: # use FW Command to setup and monitor Gearbox
        #fw_reg_wr=(14,gb_fec_test_wait)
        for slc in slices:
            sel_slice(slc)
            init_lane_for_fw(mode='pam4',input_mode='ac',lane=A_lanes)
            init_lane_for_fw(mode='nrz',input_mode='ac',lane=B_lanes)
            if '50' in gearbox_type:        
                fw_config_gearbox_50G (A_lanes,fec_b_byp=fec_b_bypass)
            else:
                fw_config_gearbox_100G(A_lanes,fec_b_byp=fec_b_bypass)
            
        for slc in slices:
            sel_slice(slc)
            start_time=time.time()   # start timer to get gearbox-ready time
            #fw_adapt_wait(max_wait=10, lane=A_lanes+B_lanes, print_en=1)
            for ln in range(16):
                pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
            print("\n...Waiting for FEC Status Lock (fec_wait=%d) ...  "%(fw_reg_rd(14))),

    else: # use FW to adapt lanes and use software to setup and monitor Gearbox
        for slc in slices:
            sel_slice(slc)
            opt_lane (mode='pam4',input_mode='ac',lane=A_lanes)
            opt_lane (mode='nrz' ,input_mode='ac',lane=B_lanes)
            if '50' in gearbox_type:        
                sw_config_gearbox_50G (A_lanes,fec_b_byp=fec_b_bypass)
            else:
                sw_config_gearbox_100G(A_lanes,fec_b_byp=fec_b_bypass)        
            start_time=time.time()   # start timer to get gearbox-ready time
            for ln in range(16):
                pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)          
        
    #for slc in slices:
    #    sel_slice(slc)
    #    fw_gearbox_wait (max_wait=10, print_en=1)
    #for slc in slices:
    #    sel_slice(slc)
    #    fw_serdes_params(lane=lanes)
    #for slc in slices:
    #    sel_slice(slc)
    #    fec_status()
####################################################################################################
#
# Baldeagle Boards in Gearbox mode, , when Falcon is the NRZ traffic source on B-side
#
####################################################################################################
def config_baldeagle_gearbox_falcon_orig(slice=0, A_lanes=None, gearbox_type='100G-1', gearbox_by_fw=True, fec_b_bypass=False):
    
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                   #Slice  [A0.............A7, B0,..........B7]
    RxPolarityMap.append([]); RxPolarityMap[0]=[1,1,0,0,1,1,0,0,   0,1,0,1,1,0,0,0] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,1,0,0,0,0,   1,0,0,0,1,1,1,1] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
    config_baldeagle_gearbox(slice, A_lanes, gearbox_type, gearbox_by_fw, fec_b_bypass)
####################################################################################################
#
# Falcon-to-Baldeagle Boards in Gearbox mode (this is for Falcon+Baldeagle+DR4)
#
####################################################################################################
def config_baldeagle_gearbox_falcon(slice=[0,1], A_lanes=[0,1,4,5], gearbox_type='100G-1', gearbox_by_fw=True, fec_b_bypass=False):
    
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                       #Slice  [A0.............A7, B0,..........B7]
    # For New Falcon-DR4
    # RxPolarityMap.append([]); RxPolarityMap[0]=[1,1,0,0,1,1,0,0,   0,1,1,0,0,1,1,0] # Slice 0 lanes, RX Polarity
    # TxPolarityMap.append([]); TxPolarityMap[0]=[0,0,0,0,0,0,0,0,   0,1,1,1,0,0,0,0] # Slice 0 lanes, TX Polarity
    # RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,1,0,0,1,1,1,0] # Slice 1 lanes, RX Polarity
    # TxPolarityMap.append([]); TxPolarityMap[1]=[1,0,1,0,1,0,1,0,   0,1,1,0,0,0,0,1] # Slice 1 lanes, TX Polarity
    # For Old Falcon
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[0]=[1,1,0,0,1,1,0,0,   1,0,1,0,0,1,1,1] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,1,0,0,0,0,   1,0,0,0,1,1,1,1] # Slice 1 lanes, RX Polarity
    config_baldeagle_gearbox(slice, A_lanes, gearbox_type, gearbox_by_fw, fec_b_bypass)
####################################################################################################
#
# Baldeagle Boards in Gearbox mode, when another Baldeagle (rev B0) is the PAM4 traffic source on A-side
#
#
####################################################################################################
def config_baldeagle_gearbox_B0_traffic_source(slice=0, A_lanes=None, gearbox_type='100G-1', gearbox_by_fw=True, fec_b_bypass=False):
    
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                   #Slice  [A0.............A7, B0,..........B7]
    RxPolarityMap.append([]); RxPolarityMap[0]=[1,1,0,0,0,0,0,0,   0,0,1,0,0,1,1,1] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[1,1,0,0,0,0,0,0,   0,0,1,0,0,1,1,1] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
    config_baldeagle_gearbox(slice, A_lanes, gearbox_type, gearbox_by_fw, fec_b_bypass)
    

####################################################################################################
# Initialize and Optimize Baldeagle in BITMUX mode
# 
# This function configures and Optimizes Baldeagle A-lanes in NRZ 20G and B lanes in 10G modes
#
# It assumes QSFP cables connected between: Slice 0: cable between A0-3 to A4-7 
#                                           Slice 0: loopback modules on B0-7 
#                                           Slice 1: cable between A0-3 to A4-7
#                                           Slice 1: loopback modules on B0-7
#
# The main idea behind the bitmux setup is to make sure all three receivers (the one on the A side and two on the B side) are ready together. If one or more are not ready, the traffic on both directions (A-to-B and B-to-A) stops.
#
# The bitmux setup sequence:
# 1. Call any software init to the lanes going to be set up in bitmux mode (do not call software init to the lanes from this step forward)
# 2. Enable Bitmux mode configuration
# 3. Issue Configure Commands to the lanes, 20G and 10G (and start adaptation)
# 4. while (!rxAdaptationComplete ) ( timeout = 2sec ) --- Stay in the while loop until all three lanes adaptations done
#      Keep PRBS Generator ON (this is needed if Credo is TX source, such as loopback module or cable between two ports)
#      On timeout, issue HW lane-reset (or go back to step 3 and reconfigure the lanes) 
# 5. All three lanes adaptation complete ---> Disable PRBS Generator
#
####################################################################################################
def config_baldeagle_bitmux(slice=0, A_lanes=[0,1,2,3], bitmux_type='20G',print_en=1):

    sel_slice(slice)
    global gLane
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                      #Slice  [A0.............A7, B0,..........B7]
    # RxPolarityMap.append([]); RxPolarityMap[0]=[1,1,0,0,1,0,1,0,   0,0,1,0,0,1,1,1] # Slice 0 lanes, RX Polarity
    # TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    # RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 1 lanes, RX Polarity
    # TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
    
    #RxPolarityMap.append([]); RxPolarityMap[0]=[1,1,0,0,0,0,0,0,   0,1,1,0,0,1,1,0] # Slice 0 lanes, RX Polarity
    #TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    #RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 1 lanes, RX Polarity
    #TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
    #### edit by Jeff for Falcon ####
    TxPolarityMap.append([]); TxPolarityMap[0]=[0,0,0,0,0,0,0,0,   0,1,1,1,0,0,0,0] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[0]=[1,1,0,0,1,1,0,0,   0,1,1,0,0,1,1,0] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,0,1,0,1,0,1,0,   0,1,1,0,0,0,0,1] # Slice 1 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,1,0,0,1,1,1,0] # Slice 1 lanes, RX Polarity
    
    #for ln in gLane: # Set Polarity of each lane according the pol_list
        #pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
    # For Bitmux mode, 3 options supported for A-Lane groups 
    group1_bitmux=[0,1] # A_lanes group 1 -> [A0,A1] <-> [B0,B1,B2,B3]
    group2_bitmux=[2,3] # A_lanes group 2 -> [A2,A3] <-> [B4,B5,B6,B7]
    group3_bitmux=[4,5] # A_lanes group 3 -> [A4,A5] <-> [B4,B5,B6,B7]
    #Determine the corresponding B-Lanes for each group of A-Lanes
    group1_selected=0
    group2_selected=0
    B_lanes=[]
    if all(elem in A_lanes for elem in group1_bitmux):  # If A_lanes contains [0,1]
        B_lanes +=[8,9,10,11]
        group1_selected=1
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [2,3]
        B_lanes+=[12,13,14,15]
        group2_selected=1
    elif all(elem in A_lanes for elem in group3_bitmux): # If A_lanes contains [4,5]
        B_lanes+=[12,13,14,15]
        group2_selected=1
    if group1_selected==0 and group2_selected==0:
        print("\n*** Bitmux Setup: Invalid Target A-Lanes specified!"),
        print("\n*** Options: A_lanes=[0,1]"),
        print("\n***          A_lanes=[2,3]"),
        print("\n***          A_lanes=[4,5]"),
        print("\n***          A_lanes=[0,1,2,3]"),
        print("\n***          A_lanes=[0,1,4,5]"),
        return
        
    lanes = get_lane_list(lane=A_lanes+B_lanes)
    gLane=lanes    
    slices = slice_power_up_init(slice)

    #init_lane (mode='nrz20',input_mode='ac',lane=A_lanes)
    #init_lane (mode='nrz10',input_mode='ac',lane=B_lanes)
    #global gPrbsEn; gPrbsEn=False
    if '53' in bitmux_type:        
        init_lane_for_fw('pam4',input_mode='ac',lane=A_lanes)
    elif '51' in bitmux_type:
        init_lane_for_fw('pam4',input_mode='ac',lane=A_lanes)
    else:
        init_lane_for_fw('nrz',input_mode='ac',lane=A_lanes)
        
    init_lane_for_fw('nrz',input_mode='ac',lane=B_lanes)

    #for ln in lanes: # Set Polarity of each lane according the pol_list
    for ln in range(16):
        pol(TxPolarityMap[slice][ln],RxPolarityMap[slice][ln],ln,0)

    prbs_mode_select(lane=lanes, prbs_mode='functional')


    if '53' in bitmux_type:        
        fw_config_bitmux_53G(A_lanes)
    elif '51' in bitmux_type:
        fw_config_bitmux_51G(A_lanes)
    else:
        fw_config_bitmux_20G(A_lanes)
    
    prbs_mode_select(lane=lanes, prbs_mode='functional')
    #pll_cap(81,150,lane=14)
    fw_bitmux_wait(lane=A_lanes, max_wait=10, print_en=print_en)

    prbs_mode_select(lane=lanes, prbs_mode='functional')
    #auto_pol(tx_prbs='dis') # Do not enable TX PRBS Gen if in RETIMER mode, for auto polarity correction
    fw_serdes_params(lane=lanes)
    rx_monitor(rst=1,lane=lanes)
####################################################################################################
# Initialize and Optimize Baldeagle in BITMUX mode
# 
# This function configures and Optimizes Baldeagle A-lanes in NRZ 20G and B lanes in 10G modes
#
# It assumes QSFP cables connected between: Slice 0: cable between A0-3 to A4-7 
#                                           Slice 0: loopback modules on B0-7 
#                                           Slice 1: cable between A0-3 to A4-7
#                                           Slice 1: loopback modules on B0-7
#
# The main idea behind the bitmux setup is to make sure all three receivers (the one on the A side and two on the B side) are ready together. If one or more are not ready, the traffic on both directions (A-to-B and B-to-A) stops.
#
# The bitmux setup sequence:
# 1. Call any software init to the lanes going to be set up in bitmux mode (do not call software init to the lanes from this step forward)
# 2. Enable Bitmux mode configuration
# 3. Issue Configure Commands to the lanes, 20G and 10G (and start adaptation)
# 4. while (!rxAdaptationComplete ) ( timeout = 2sec ) --- Stay in the while loop until all three lanes adaptations done
#      Keep PRBS Generator ON (this is needed if Credo is TX source, such as loopback module or cable between two ports)
#      On timeout, issue HW lane-reset (or go back to step 3 and reconfigure the lanes) 
# 5. All three lanes adaptation complete ---> Disable PRBS Generator
#
####################################################################################################
def config_baldeagle_bitmux_no_fw(Slice=None, A_lanes=[],print_en=1):

    slices = slice_power_up_init(Slice)

    
    ########################## QSFP Board's RX and TX Polarities for Slice 0 and 1
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                      #Slice  [A0.............A7, B0,..........B7]
    RxPolarityMap.append([]); RxPolarityMap[0]=[ 1, 1, 0, 0, 1, 1, 0, 1,   0, 0, 1, 0, 0, 1, 1, 1 ] # Slice 0 lanes, RX Polarity
   #RxPolarityMap.append([]); RxPolarityMap[0]=[ 1, 1, 1, 1, 1, 1, 1, 1,   0, 0, 1, 0, 1, 1, 1, 1 ] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[ 1, 1, 1, 1, 1, 1, 1, 1,   1, 1, 1, 1, 1, 1, 1, 1 ] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[ 1, 1, 1, 1, 1, 1, 1, 0,   0, 0, 1, 0, 0, 1, 1, 1 ] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[ 1, 1, 1, 1, 1, 1, 1, 1,   1, 1, 1, 1, 1, 1, 1, 1 ] # Slice 1 lanes, TX Polarity
    ########################## QSFP Board's TX Source for each lane's RX (for Slice 0 and 1)
    
    ### 0 ### For this board, we use A4-7 as 20G TX Source to A0-3)
    #tx_taps(0,-8,17,0,0,lane='all')
    #nrz_lanes_20_tx_source=[4,5,6,7]
    #init_lane (mode='nrz20',input_mode='ac',lane=nrz_lanes_20_tx_source)
    #opt_lane (mode='nrz20',input_mode='ac',lane=range(0,8))
    #opt_lane (mode='nrz10',input_mode='ac',lane=range(8,16))
    #print("BITMUX Config: Initialization of ALL Lanes Done.")
    
    ### 1 ### Initialize the target lanes, NO adaptation yet (A:20G and B:10G)
    print("BITMUX: Initializing A-Lanes in 20G and B-Lanes in 10G.")
    nrz_lanes_20, nrz_lanes_10 = sw_config_retimer(A_lanes) 
    global gLane; gLane = sorted(list(set(nrz_lanes_20 + nrz_lanes_10)))
    tx_taps(0,-8,17,0,0,lane=nrz_lanes_20+nrz_lanes_10)
    opt_lane (mode='nrz20',input_mode='ac',lane=nrz_lanes_20)
    opt_lane (mode='nrz10',input_mode='ac',lane=nrz_lanes_10)
    print("BITMUX: Done Initialing A-Lanes in 20G and B-Lanes in 10G.")
        
    ### 2 ### BITMUX mode top registers
    print("BITMUX: Setting up Bitmux mode. ALL Lanes in FUNCTIONAL mode."),
    sw_config_bitmux_20G(A_lanes,print_en)
    print("Done")
    
    ### 3 ### ADAPT the lanes used in this bitmux configuraiton, NO initialization by python any more
    loop=1; adapt_timeout=2.0
    #while(1):
    #print("BITMUX: Adapting all Bitmux lanes.")
    fw_config_lane(mode='nrz20',lane=nrz_lanes_20)
    fw_config_lane(mode='nrz10',lane=nrz_lanes_10)
    #print("BITMUX: Done adapting all Bitmux lanes.")

    bitmux_workaround = 'en'
    if bitmux_workaround == 'en':
        print("BITMUX: Checking ALL lanes are adapted. If not, switching TX to PRBS mode.")
        start_time=time.time()
        while( fw_adapt_wait (max_wait=adapt_timeout, lane=gLane, print_en=1)[0]==0):
            print("BITMUX: Timed out waiting for ALL lanes to adapt. Switching TX to PRBS mode. Loop # %d "%(loop))
            prbs_mode_select(lane=gLane, prbs_mode='prbs')
            loop+=1
            if loop>3: break
   
    #print("BITMUX: Making sure all lanes' TX are in FUNCTIONAL mode.")
    prbs_mode_select(lane=gLane, prbs_mode='functional')
    
    fw_config_lane()
    serdes_params()
    #auto_pol(print_en=1)
    rx_monitor(rst=1)
####################################################################################################
def config_smk(Slice=0, mode='nrz', input_mode='dc', lane=None):
    lanes = get_lane_list(lane)
    global gLane; gLane= lanes
    
    ########################## 15x15 Board's RX and TX Polarities for Slice 0 and 1
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                  #Slice  [A0.............A7, B0,..........B7]
                                   #   Lane No: [A0.A1.A2.A3.A4.A5.A6.A7,B0.B1.B2.B3.B4.B5.B6.B7 ] 
    RxPolarityMap.append([]); RxPolarityMap[0]=[ 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0 ] # Slice 0 lanes, RX Polarity
 #   RxPolarityMap.append([]); RxPolarityMap[0]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
    ################################################################################################
                                   #   Lane No: [A0.A1.A2.A3.A4.A5.A6.A7,B0.B1.B2.B3.B4.B5.B6.B7 ] 
    RxPolarityMap.append([]); RxPolarityMap[0]=[ 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0 ] # Slice 0 lanes, RX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[ 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0 ] # Slice 0 lanes, RX Polarity

    ################################################################################################

    mdio('connect',Slice)
    slice_reset()
    set_bandgap('on', lanes) 
    set_top_pll(pll_side='both', freq=195.3125)

    #if not fw_loaded():
    #    fw_load()
    
    ######## Initialize and adapt lanes in PAM4 or NRZ mode
    if Slice == 0: # Slice 0      
        ########## DEVICE 0, In SMK PCBA lanes A4-A7 and B4-B7 are looped back by traces
        pcb_traces_opt_en=0
        if pcb_traces_opt_en:
            pcb_trace_lanes = [4,5,6,7, 12,13,14,15]
            opt_lane(mode=mode,input_mode='dc',lane=pcb_trace_lanes) 
        else:
            pcb_trace_lanes=[]
           
        ########## DEVICE 0 A0-A3, B0-B3 can be connected to external Channels
        opt_lane (mode=mode,input_mode=input_mode,lane=lanes) 
        
    else: ########## DEVICE 1, In SMK PCBA, All B-Lanes are Looped back by PCBA Traces       
        opt_lane(mode=mode,input_mode='dc',lane='all')
        
    serdes_params(lane=lanes)
    rx_monitor(rst=1,lane=lanes)
    if fw_loaded(print_en=0): fw_config_lane()

    
####################################################################################################
def opt_tx_taps(slice=0, mode='nrz', input_mode='ac', lane=None):
    global gLaneStats #[per Slice][per lane], [PrbsCount, PrbsCount-1,PrbsCount-2, PrbsRstTime, PrbsLastReadoutTime]
    pre2 = 2
    pre1 = -8
    main = 17
    post1 = 0
    post2 = 0
    tx_taps(pre2, pre1, main, post1, post2)
    Eye_best = eye_pam4(lane)
    Ber_best, PostBerBest = ber(lane,rst=1,t=5)[lane]
    pre2_best = pre2
    pre1_best = pre1
    main_best = main
    post1_best = post1
    post2_best = post2
    pre2_next = pre2
    pre1_next = pre1
    main_next = main
    post1_next = post1
    post2_next = post2
    
    for x in range(-6,6): #pre2 range
        tx_taps(x, pre1_next, main_next, post1_next, post2_next)
        if mode == 'pam4':
            Eye_post = eye_pam4(lane)
        elif mode == 'nrz':
            Eye_post =  eye_nrz(lane)
        Ber_post, PostFecBer = ber(lane,rst=1,t=5)[lane]    
        serdes_params(lane)  
        time.sleep(1)            
        if (Eye_post > Eye_best and Ber_post < Ber_best):
            Eye_best = Eye_post
            Ber_best = Ber_post
            pre2_best = x
        print('\nBest pre 2 setting so far is : %d' %pre2_best)
        pre2_next = x
        #serdes_params()
        #time.sleep(1)
        
        for x in range(-12,0): #pre1 range
            tx_taps(pre2_next, x, main_next, post1_next, post2_next)
            if mode == 'pam4':
                Eye_post = eye_pam4(lane)
            elif mode == 'nrz':
                Eye_post =  eye_nrz(lane)
            Ber_post, PostFecBer = ber(lane,rst=1,t=5)[lane]    
            serdes_params(lane)  
            time.sleep(1)            
            if (Eye_post > Eye_best and Ber_post < Ber_best):
                Eye_best = Eye_post
                Ber_best = Ber_post
                pre1_best = x
            print('\nBest pre 1 setting so far is : %d' %pre1_best)
            pre1_next = x
            #serdes_params()
            #time.sleep(1)   
            
            for x in range(-16,21): #main range
                tx_taps(pre2_next, pre1_next, x, post1_next, post2_next)
                if mode == 'pam4':
                    Eye_post = eye_pam4(lane)
                elif mode == 'nrz':
                    Eye_post =  eye_nrz(lane)
                Ber_post,PostFecBer = ber(lane,rst=1,t=5)[lane]    
                serdes_params(lane)  
                time.sleep(1)            
                if (Eye_post > Eye_best and Ber_post < Ber_best):
                    Eye_best = Eye_post
                    Ber_best = Ber_post
                    main_best = x
                print('\nBest main setting so far is : %d' %main_best)
                main_next = x
                #serdes_params()
                #time.sleep(1)            
            
                for x in range(-7,0): #post1 range
                    tx_taps(pre2_next, pre1_next, main_next, x, post2)
                    if mode == 'pam4':
                        Eye_post = eye_pam4(lane)
                    elif mode == 'nrz':
                        Eye_post =  eye_nrz(lane)
                    Ber_post,PostFecBer = ber(lane,rst=1,t=5)[lane]    
                    serdes_params(lane)  
                    time.sleep(1)            
                    if (Eye_post > Eye_best and Ber_post < Ber_best):
                        Eye_best = Eye_post
                        Ber_best = Ber_post
                        post1_best = x
                    print('\nBest post1 setting so far is : %d' %post1_best)
                    post1_next = x
                        #time.sleep(1)
                
                    for x in range(-3,3): #post2 range
                        tx_taps(pre2_next, pre1_next, main_next, post1_next, x)
                        if mode == 'pam4':
                            Eye_post = eye_pam4(lane)
                        elif mode == 'nrz':
                            Eye_post =  eye_nrz(lane)
                        Ber_post,PostFecBer = ber(lane,rst=1,t=5)[lane]    
                        serdes_params(lane)  
                        time.sleep(1)            
                        if (Eye_post > Eye_best and Ber_post < Ber_best):
                            Eye_best = Eye_post
                            Ber_best = Ber_post
                            post2_best = x
                        print('\nBest post2 setting so far is : %d' %post2_best)
                        post2_next = x
                
    print ('\nOptimal precursor 2 value is: %d' %pre2_best)
    print ('\nOptimal precursor 1 value is: %d' %pre1_best)
    print ('\nOptimal main cursor value is: %d' %main_best)
    print ('\nOptimal post cursor 1 value is: %d' %post1_best)
    print ('\nOptimal post cursor 2 value is: %d' %post2_best)
      
    tx_taps(pre2, pre1, main, post1, post2)
    serdes_params(lane)
####################################################################################################
def opt_tx_taps_orig(slice=0, mode='nrz', input_mode='ac', lane=None):
    
    lanes = get_lane_list(lane) 
    slices = get_slice_list(slice)

    for slc in slices:
        init_lane (mode=mode,input_mode=input_mode,lane=lanes) 

    for slc in slices:
        sel_slice(slc)

        fw_config_lane (mode=mode,lane=lanes)
        fw_adapt_wait (max_wait=None, lane=lanes, print_en=1)
        fw_serdes_params(lane=lanes)
        rx_monitor(rst=1,lane=lanes)
  
    sel_slice(slices[0])

    
####################################################################################################
def config_15x15(mode='nrz', input_mode='dc', lane=None):
    
    ########################## 15x15 Board's RX and TX Polarities for Slice 0 and 1
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                  #Slice  [A0.............A7, B0,..........B7]
    RxPolarityMap.append([]); RxPolarityMap[0]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
    ################################################################################################
 
    lanes = get_lane_list(lane)
    # Turn off all the lanes first and later turn on only lanes under test
    #mdio('connect',0) # Slice is always = 0
    set_bandgap('off', 'all')
    #slice_reset()
    set_bandgap('on', lanes) 
    #slice_reset()
    set_top_pll(pll_side='both', freq=195.3125)

    # if not fw_loaded():
        # fw_load()
    
    ########################## QSFP Board's Lanes defined here
    #global gNrzLanes;  gNrzLanes = [0,1,2,3] #[0,1,2,3,4,5,6,7]
    #global gPam4Lanes; gPam4Lanes= [0,1,2,3] #[8,9,10,11,12,13,14,15]
    #global gLane;      gLane=gPam4Lanes
    
    opt_lane(mode=mode,input_mode=input_mode,lane=lanes)
    for ln in gLane: # Set Polarity of each lane according the pol_list
        pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
        
    data = serdes_params(lane=lanes)
    rx_monitor(rst=1,lane=lanes)
    return data

####################################################################################################
def config_DAC(Slice=0, mode=None, qsfp_type='QSFP-DDx2'):
    global gSlice
    global gLane
    
    ####  above TX-to-RX connection makes sure the opt() routine does not change its own TX driving the RX of the other board
    ########################## 15x15 QSFP-DD Board's RX and TX Polarities for Slice 0 and 1
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                      #Slice  [A0.............A7, B0,..........B7]
    RxPolarityMap.append([]); RxPolarityMap[0]=[0,0,0,0,0,0,0,0,   1,1,1,1,1,1,1,1] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
    ####################################################################################################### 
     
    
    ########################## QSFP Board's Lanes defined here
    global gPam4Lanes
    global gNrzLanes
    if qsfp_type=='QSFP-DDx2':
        gNrzLanes = range(16) 
        gPam4Lanes= range(16) 
    elif qsfp_type=='QSFP-DD':
        gNrzLanes = [0,1,2,3,4,5,6,7]
        gPam4Lanes= [0,1,2,3,4,5,6,7] 
    elif qsfp_type=='QSFPx2':
        gNrzLanes = [0,3,5,6,8,9,10,11,12,13,14,15]
        gPam4Lanes= [0,3,5,6,8,9,10,11,12,13,14,15]     
    else: # qsfp_type=='QSFP':
        gNrzLanes = [0,3,5,6]
        gPam4Lanes= [0,3,5,6] 
        
    gLane= gPam4Lanes
    
    ########################## Turn off Unused lanes to save power
    # in case of DDx2, no lane is off because all 16 lanes used
    #set_bandgap('off', lane='all')
    set_bandgap( 'on', lane=gPam4Lanes)
    set_bandgap( 'on', lane=gNrzLanes)
    set_top_pll(pll_side='both', freq=195.3125)
    
    if not fw_loaded():
        fw_load()

    ######## Set up the PAM4 lanes
    if mode.upper()=='PAM4':
        tx_taps(+3,-12,18,-2,0,lane=gPam4Lanes)
        #tx_taps(+3,-14,22,0,0,lane=gPam4Lanes))
        opt_lane (mode=mode,input_mode=input_mode,lane=gPam4Lanes)
        for ln in gPam4Lanes: # Set Polarity of each lane according the pol_list
            pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
        
    ######## Set up the NRZ lanes
    elif mode.upper()=='NRZ':
        tx_taps(+3,-12,17,0,0,lane=gNrzLanes)
        tx_taps(+3,-14,22,0,0,lane=gNrzLanes)
        opt_lane (mode=mode,input_mode=input_mode,lane=gNrzLanes) 
        for ln in gNrzLanes: # Set Polarity of each lane according the pol_list
            pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
        
    ######## If mode=None, do not force to initialize and optimize the lanes and leave them as they are
    else:
        for ln in gPam4Lanes: # Set Polarity of each lane according the pol_list
            pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
    
    serdes_params(lane=gLane)
####################################################################################################
def config_tm3(Slice=0, mode=None, input_mode='ac'):
    
    ########################## QSFP Board's Lanes defined here
    global gNrzLanes;  gNrzLanes = [0,1]
    global gPam4Lanes; gPam4Lanes= [0,1]
    global gLane; gLane = gPam4Lanes
    
    ########################## TM3 box RX and TX Polarities for Slice 0 and 1
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                      #Slice  [A0.............A7, B0,..........B7]
    RxPolarityMap.append([]); RxPolarityMap[0]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
    ##########################
    
    mdio('connect',Slice)
    if mode!=None: slice_reset()
    set_bandgap('on', 'all') 
    set_top_pll(pll_side='both', freq=195.3125)
    get_lane_mode('all')
       
    ######## Set up the PAM4 lanes
    if mode==None:
        for ln in gNrzLanes: # Set Polarity of each lane according the pol_list
            pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
    
    ######## Set up the PAM4 lanes
    elif mode=='pam4':
        init_lane(mode=mode,input_mode=input_mode,lane=gPam4Lanes)
        tx_taps(4,-12,23,0,0,lane=gNrzLanes) 
        opt_lane (mode=mode,input_mode=input_mode,lane=gPam4Lanes)
        for ln in gPam4Lanes: # Set Polarity of each lane according the pol_list
            pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
         
    ######## Set up the NRZ lanes
    elif mode=='nrz':
        init_lane(mode=mode,input_mode=input_mode,lane=gNrzLanes) 
        tx_taps(4,-12,23,0,0,lane=gNrzLanes)
        opt_lane (mode=mode,input_mode=input_mode,lane=gNrzLanes) 
        for ln in gNrzLanes: # Set Polarity of each lane according the pol_list
            pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)

    tx_taps(4,-12,23,0,0,lane=gNrzLanes) # TX for tm3 chip
    serdes_params(lane=gLane)
    rx_monitor(rst=1,lane=gLane)

####################################################################################################
# BE QSFP board connected to Blackhawk Box
#
# A0, A1 Lanes     : in PAM4 mode connected to Blackhawk
# B0,B1,B2,B3 Lanes: in NRZ mode looping back to themselves
# 
# Usage 1: Gearbox 50G : A0 to B0/B1
#          config_qsfp_blackhawk(Slice=0, A_lanes=[0], B_lanes=[8,9], mode='50g')
#          Start from scratch. Adapt all 3 lanes and set up Gearbox 50G mdoe
#
# Usage 2: Gearbox 100G : A0/A1 to B0/B1/B2/B3
#          config_qsfp_blackhawk(Slice=0, A_lanes=[0,1], B_lanes=[8,9,10,11], mode='100g')
#          Start from scratch. Adapt all 3 lanes and set up Gearbox 50G mdoe
#
# Usage 3: Only Gearbox 50G or 100G without Adaptation 
#          config_qsfp_blackhawk(Slice=0, A_lanes=[0], B_lanes=[8,9], mode=None)
#          config_qsfp_blackhawk(Slice=0, A_lanes=[0,1], B_lanes=[8,9,10,11], mode=None)
#
####################################################################################################
def config_qsfp_blackhawk(Slice=0, A_lanes=[0,1], B_lanes=[8,9,10,11], mode=None):
  
    ########################## QSFP board to Blackhawk box RX and TX Polarities for Slice 0 and 1
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                      #Slice  [A0.............A7, B0,..........B7]
    RxPolarityMap.append([]); RxPolarityMap[0]=[1,0,0,0,1,0,1,0,   0,0,1,0,0,0,0,0] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,0,1,1,1,0,0,0,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
    ##########################    
    
    global gPam4TxSourceIsCredo;   gPam4TxSourceIsCredo =0 # 0: means do not touch PAM4 lane's TX taps, as it's driving another NRZ lane's RX

    global gNrzLanes;   gNrzLanes = B_lanes
    global gPam4Lanes;  gPam4Lanes= A_lanes
    global gLane;       gLane=sorted(list(set(gNrzLanes+gPam4Lanes)))
    #global gPrbsEn;     gPrbsEn=False
    
    mdio('connect',Slice)
    #slice_reset()
    set_bandgap('on', gLane) 
    set_top_pll(pll_side='both', freq=195.3125)
              
    ######## Set up and opt lanes
    if mode!=None:
    
        if not fw_loaded():
            fw_load()
                
        ######## Set up the PAM4 lanes, connected to Spirent
        init_lane(mode='pam4',input_mode='ac',lane=gPam4Lanes)
        opt_lane (mode='pam4',input_mode='ac',lane=gPam4Lanes)
        #######################  PAM4 Functional mode TX
        for ln in gPam4Lanes:
            tx_taps(3,-12,23,0,0,lane=ln) # good for BH
            wreg(0x0a0,0xe320,ln) # PAM4 mode, PRBS clock en before Patt en
            wreg(0x0a0,0x8320,ln) # PAM4 mode, TX pam4 FUNCTIONAL MODE
            wreg(0x0af,0xfa08,ln) # PAM4 mode, tx_automode| MSB first|gc=en|Pre off|SHmode

        ######## Set up the NRZ lanes, they are in loopback-mode
        init_lane(mode='nrz',input_mode='ac',lane=gNrzLanes) 
        opt_lane (mode='nrz',input_mode='ac',lane=gNrzLanes) 
        #######################  NRZ Functional mode TX    
        for ln in gNrzLanes:
            wreg(0x0a0,0x0120,ln) # NRZ mode, TX functional mode
            wreg(0x0a0,0x0120,ln) # NRZ mode, TX functional mode
            wreg(0x0a0,0x0120,ln) # NRZ mode, TX functional mode
            
    else: # if mode==None:
        if not fw_loaded(print_en=0):
            lr()
            
    # Before doing Gearbox, set Polarity of each lane according the pol_list        
    for ln in gLane: 
        pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)  
            
    ########################## GearBox and FEC set up here
    if len(A_lanes) > 1: #if mode=='100g' or  mode=='100G':
        config_gearbox_mode_100G(A_lanes=gPam4Lanes, B_lanes=gNrzLanes)    
    else:                #if mode=='50g' or  mode=='50G':
        config_gearbox_mode_50G(A_lanes=gPam4Lanes, B_lanes=gNrzLanes)
        
    serdes_params()
    fec_status()
       
####################################################################################################
# BE Ardent board connected to Blackhawk Box
#
# A0, A1 Lanes     : in PAM4 mode connected to Blackhawk
# B0,B1,B2,B3 Lanes: in NRZ mode looping back to themselves
# 
# Usage 1: Gearbox 50G : A0 to B0/B1
#          config_ardent_blackhawk(Slice=0, A_lanes=[0], B_lanes=[8,9], mode='50g')
#          Start from scratch. Adapt all 3 lanes and set up Gearbox 50G mdoe
#
# Usage 2: Gearbox 100G : A0/A1 to B0/B1/B2/B3
#          config_ardent_blackhawk(Slice=0, A_lanes=[0,1], B_lanes=[8,9,10,11], mode='100g')
#          Start from scratch. Adapt all 3 lanes and set up Gearbox 50G mdoe
#
# Usage 3: Only Gearbox 50G or 100G without Adaptation 
#          config_ardent_blackhawk(Slice=0, A_lanes=[0], B_lanes=[8,9], mode=None)
#          config_ardent_blackhawk(Slice=0, A_lanes=[0,1], B_lanes=[8,9,10,11], mode=None)
#
####################################################################################################
def config_ardent_blackhawk(Slice=0, A_lanes=[0,1], B_lanes=[8,9,10,11], mode=None):
  
    ########################## Ardent board to Blackhawk box RX and TX Polarities for Slice 0 and 1
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                      #Slice  [A0.............A7, B0,..........B7]
    RxPolarityMap.append([]); RxPolarityMap[0]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
    ##########################    
    
    global gPam4TxSourceIsCredo;   gPam4TxSourceIsCredo =0 # 0: means do not touch PAM4 lane's TX taps, as it's driving another NRZ lane's RX

    global gNrzLanes;   gNrzLanes = B_lanes
    global gPam4Lanes;  gPam4Lanes= A_lanes
    global gLane;       gLane=sorted(list(set(gNrzLanes+gPam4Lanes)))
    #global gPrbsEn;     gPrbsEn=False
    
    mdio('connect',Slice)
    #slice_reset()
    set_bandgap('on', 'all') 
    set_top_pll(pll_side='both', freq=195.3125)

        
    ######## Set up and opt lanes, if 
    if mode!=None:
    
        if not fw_loaded():
            fw_load()
            
        ######## Set up the PAM4 lanes, connected to Spirent
        init_lane(mode='pam4',input_mode='dc',lane=gPam4Lanes)
        opt_lane (mode='pam4',input_mode='dc',lane=gPam4Lanes)
        #######################  PAM4 Functional mode TX
        for ln in gPam4Lanes:
            tx_taps(3,-12,23,0,0,lane=ln) # good for BH
            wreg(0x0a0,0xe320,ln) # PAM4 mode, PRBS clock en before Patt en
            wreg(0x0a0,0x8320,ln) # PAM4 mode, TX pam4 FUNCTIONAL MODE
            wreg(0x0af,0xfa08,ln) # PAM4 mode, tx_automode| MSB first|gc=en|Pre off|SHmode

        ######## Set up the NRZ lanes, they are in loopback-mode
        init_lane(mode='nrz',input_mode='dc',lane=gNrzLanes) 
        opt_lane (mode='nrz',input_mode='dc',lane=gNrzLanes) 
        #######################  NRZ Functional mode TX    
        for ln in gNrzLanes:
            wreg(0x0a0,0x0120,ln) # NRZ mode, TX functional mode
            wreg(0x0a0,0x0120,ln) # NRZ mode, TX functional mode
            wreg(0x0a0,0x0120,ln) # NRZ mode, TX functional mode
            
    else: # if mode==None:
        if not fw_loaded(print_en=0):
            lr()
            
    # Before doing Gearbox, set Polarity of each lane according the pol_list        
    for ln in gLane: 
        pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)   
        
    ########################## GearBox and FEC set up here
    if len(A_lanes) > 1: #if mode=='100g' or  mode=='100G':
        config_gearbox_mode_100G(A_lanes=gPam4Lanes, B_lanes=gNrzLanes)    
    else:                #if mode=='50g' or  mode=='50G':
        config_gearbox_mode_50G(A_lanes=gPam4Lanes, B_lanes=gNrzLanes)
    
    serdes_params()
    fec_status()
    
####################################################################################################
def config_spirent(Slice=0, mode='50g'):
    global gSlice
    global gNrzLanes;  
    global gPam4Lanes; 
    
    ####################################################################################################### 
    
    ########################## SMK Board's RX and TX Polarities for Slice 0 and 1
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                     #Slice  [A0.............A7, B0,..........B7]
    RxPolarityMap.append([]); RxPolarityMap[0]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity

    # Turn off Slice we don't first (not used)
    # chip.connect(0); gSlice=0 #phy_addr = not Slice)
    # set_top_pll(pll_side='both', freq=195.3125)
    # set_bandgap('off', 'all')
    # chip.disconnect()
    
    # Initialize Slice we use, Top PLL
    # turn off all B's first and later turn only those used when calling init_lane_xxx_mode()
    #chip.connect(1); gSlice=1 #phy_addr=Slice)
    set_top_pll(pll_side='both', freq=195.3125)
    set_bandgap('off', 'all') 
    #chip.disconnect()

    #chip.connect(phy_addr=Slice); gSlice=Slice
    set_bandgap('on', 'all') 
    set_top_pll(pll_side='both', freq=195.3125)
    
    ########################## Lanes defined here
    if mode=='100g' or  mode=='100G':
        gNrzLanes = [8,9,10,11]
        gPam4Lanes= [0,1]
    elif mode=='50g' or  mode=='50G':
        gNrzLanes = [8,9,10,11]
        gPam4Lanes= [0,1]
        

    ######## Set up the PAM4 lanes, connected to Spirent
    init_lane(mode='pam4',input_mode='ac',lane=gPam4Lanes)
    opt_lane(mode='pam4',input_mode='ac',lane=gPam4Lanes)
    for ln in gPam4Lanes: # Set Polarity of each lane according the pol_list
        pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
         
    #######################  PAM4 Functional mode TX
    for ln in gPam4Lanes:
        tx_taps(0,-2,30,0,0,lane=ln)
        wreg(0x0a0,0xe320,ln) # PAM4 mode, PRBS clock en before Patt en
        wreg(0x0a0,0x8320,ln) # PAM4 mode, TX pam4 FUNCTIONAL MODE
        wreg(0x0af,0xfa08,ln) # PAM4 mode, tx_automode| MSB first|gc=en|Pre off|SHmode

    ######## Set up the NRZ lanes, they are in loopback-mode
    init_lane(mode='nrz',input_mode='dc',lane=gNrzLanes) 
    opt_lane(mode='nrz',input_mode='dc',lane=gNrzLanes) 
    for ln in gNrzLanes: # Set Polarity of each lane according the pol_list
        pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
    #######################  NRZ Functional mode TX    
    for ln in gNrzLanes:
        wreg(0x0a0,0x0120,ln) # NRZ mode, TX functional mode
        wreg(0x0a0,0x0120,ln) # NRZ mode, TX functional mode
        wreg(0x0a0,0x0120,ln) # NRZ mode, TX functional mode

        
    ########################## GearBox and FEC set up here
    if mode=='100g' or  mode=='100G':
        config_gearbox_mode_100G()    
    elif mode=='50g' or  mode=='50G':
        config_gearbox_mode_50G()
        
    serdes_params(lane='all')
    
####################################################################################################
def config_iris(Slice=0, mode='prbs_pam4'):
    global gSlice
    global gNrzLanes;  
    global gPam4Lanes; 
    global gLane;       
    global gPrbsEn;
    
    ##########################
    # Define Globals here once
    ########################## SMK Board's TX Source for each lane's RX is its own TX (assuming loop-back)
    ########################## SMK Board's RX and TX Polarities for Slice 0 and 1
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                      #Slice  [A0.............A7, B0,..........B7]
    RxPolarityMap.append([]); RxPolarityMap[0]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity

    # Turn off Slice we don't first (not used)
    # chip.connect(0); gSlice=0 #phy_addr = not Slice)
    # set_top_pll(pll_side='both', freq=195.3125)
    # set_bandgap('off', 'all')
    # chip.disconnect()
    
    # Initialize Slice we use, Top PLL
    # turn off all B's first and later turn only those used when calling init_lane_xxx_mode()
    #chip.connect(1); gSlice=1 #phy_addr=Slice)
    #slice_reset() # no need to reset the Slice that will be powered down
    mdio('connect',Slice=1)
    set_top_pll(pll_side='both', freq=195.3125)
    set_bandgap('off', 'all') 
    #chip.disconnect()

    #chip.connect(phy_addr=Slice); gSlice=Slice
    mdio('connect',Slice=Slice)
    #slice_reset()
    set_bandgap('on', 'all') 
    set_top_pll(pll_side='both', freq=195.3125)
    ##### FW LOAD to be done here
    #logic_reset() already done inside 'slice_reset()"
    
    
    ########################## Lanes and modes defined here
    if mode.lower()=='prbs_pam4':
        gPam4Lanes = range(16)#[8,9,10,11]
        gNrzLanes= []
        gPrbsEn=True
    elif mode.lower()=='prbs_nrz':
        gPam4Lanes = []
        gNrzLanes= range(16)#[8,9,10,11]
        gPrbsEn=True
    if mode.lower()=='loopback_pam4':
        gPam4Lanes = [8,9,10,11]
        gNrzLanes= []
        gPrbsEn=False
    elif mode.lower()=='loopback_nrz':
        gPam4Lanes = []
        gNrzLanes= [8,9,10,11]
        gPrbsEn=False
    elif mode.lower()=='retimer_pam4':
        gPam4Lanes = [8,9,10,11]
        gNrzLanes= []
        gPrbsEn=False
    elif mode.lower()=='retimer_nrz':
        gPam4Lanes = []
        gNrzLanes= [8,9,10,11]
        gPrbsEn=False
    elif mode.lower()=='gearbox_100g':
        gPam4Lanes = [0,1]
        gNrzLanes= [8,9,10,11]
        gPrbsEn=False
    elif mode.lower()=='gearbox_50g':
        gPam4Lanes = [0]
        gPrbsEn=False
        gNrzLanes= [8,9]
    

    
    if gPam4Lanes != []:
        gLane = gPam4Lanes  
        ######## Set up the PAM4 lanes, connected to Spirent
        init_lane_pam4(input_mode='ac',lane=gPam4Lanes)
        opt_lane_pam4 (input_mode='ac',lane=gPam4Lanes)
        for ln in gPam4Lanes: # Set Polarity of each lane according the pol_list
            pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
             
        #######################  PAM4 Functional mode TX
        tx_taps(4,-12,23,0,0,lane=gPam4Lanes)
        prbs_mode_select(prbs_mode=gPrbsEn,lane=gPam4Lanes)

    if gNrzLanes != []:
        gLane = gNrzLanes 
        ######## Set up the NRZ lanes, they are in loopback-mode
        init_lane_nrz(input_mode='ac',lane=gNrzLanes) 
        opt_lane_nrz (input_mode='ac',lane=gNrzLanes) 
        for ln in gNrzLanes: # Set Polarity of each lane according the pol_list
            pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
        #######################  NRZ Functional mode TX    
        tx_taps(4,-12,23,0,0,lane=gNrzLanes)
        prbs_mode_select(prbs_mode=gPrbsEn, lane=gNrzLanes)


        
    ########################## GearBox and FEC set up here
    # if mode.lower()=='gearbox_100g':
        # config_gearbox_mode_100G()    
    # elif mode.lower()=='gearbox_50g':
        # config_gearbox_mode_50G()

    serdes_params(lane='all')


####################################################################################################
# Initialize Baldeagle inside *new* QSFP board
# 
# This function configures Baldeagle lanes in NRZ or PAM4 modes
#
# It assumes QSFP cables connected between: Slice 0: A0-3 to A4-7 
#                                           Slice 0: B0-3 to B4-7
#                                           Slice 1: A0-3 to A4-7
#                                           Slice 1: B0-3 to B4-7
# 
#
####################################################################################################
def init_qsfp(nrz_lanes=range(8,16), pam4_lanes=range(0,8)):
    
    
    ########################## QSFP Board's Lanes defined here
    global gNrzLanes;  gNrzLanes =  nrz_lanes  
    global gPam4Lanes; gPam4Lanes= pam4_lanes
    global gLane;      gLane = sorted(list(set(pam4_lanes + nrz_lanes)))
    
    ########################## QSFP Board's RX and TX Polarities for Slice 0 and 1
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                      #Slice  [A0.............A7, B0,..........B7]
   #RxPolarityMap.append([]); RxPolarityMap[0]=[1,1,0,0,0,0,0,0,   1,0,1,0,0,1,1,1] # Slice 0 lanes, RX Polarity
    RxPolarityMap.append([]); RxPolarityMap[0]=[ 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0 ] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
   #RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,1,1,   0,0,1,0,1,0,0,1] # Slice 1 lanes, RX Polarity
                              #   Lane No: [A0.A1.A2.A3.A4.A5.A6.A7,B0.B1.B2.B3.B4.B5.B6.B7 ] 
    RxPolarityMap.append([]); RxPolarityMap[1]=[ 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1 ] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity

  # Original QSFP Board with different RX Polarity mapping
                                      #Slice  [A0.............A7, B0,..........B7]
  # RxPolarityMap.append([]); RxPolarityMap[0]=[0,0,0,0,1,0,0,0,   0,1,0,0,0,0,0,1] # Slice 0 lanes, RX Polarity
  # TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
  # RxPolarityMap.append([]); RxPolarityMap[1]=[1,0,0,0,1,0,1,0,   0,1,1,0,0,0,1,1] # Slice 1 lanes, RX Polarity
  # TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
     
    for ln in gLane: # Set Polarity of each lane according the pol_list
        pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
            
    ##########################
    get_lane_mode('all')
    #rx_monitor(rst=1, print_en=0)

####################################################################################################
# Initialize Baldeagle inside *new* QSFP board
# 
# This function configures Baldeagle lanes in NRZ or PAM4 modes
#
# It assumes QSFP cables connected between: Slice 0: A0-3 to A4-7 
#                                           Slice 0: B0-3 to B4-7
#                                           Slice 1: A0-3 to A4-7
#                                           Slice 1: B0-3 to B4-7
# 
#
####################################################################################################
def init_qsfp_bh(nrz_lanes=range(8,16), pam4_lanes=range(0,8)):
    
    
    ########################## QSFP Board's Lanes defined here
    global gNrzLanes;  gNrzLanes =  nrz_lanes  
    global gPam4Lanes; gPam4Lanes= pam4_lanes
    global gLane;      gLane = sorted(list(set(pam4_lanes + nrz_lanes)))
    
    ########################## QSFP Board's RX and TX Polarities for Slice 0 and 1
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                      #Slice  [A0.............A7, B0,..........B7]
    RxPolarityMap.append([]); RxPolarityMap[0]=[1,0,0,0,0,0,0,0,   0,0,1,0,0,1,1,1] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,1,1,   0,0,1,0,1,0,0,1] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity

  # Original QSFP Board with different RX Polarity mapping
                                      #Slice  [A0.............A7, B0,..........B7]
  # RxPolarityMap.append([]); RxPolarityMap[0]=[0,0,0,0,1,0,0,0,   0,1,0,0,0,0,0,1] # Slice 0 lanes, RX Polarity
  # TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
  # RxPolarityMap.append([]); RxPolarityMap[1]=[1,0,0,0,1,0,1,0,   0,1,1,0,0,0,1,1] # Slice 1 lanes, RX Polarity
  # TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity
   
    for ln in gLane: # Set Polarity of each lane according the pol_list
        pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
            
    ##########################
    get_lane_mode('all')
    #rx_monitor(rst=1, print_en=0)
    

####################################################################################################
# Initialize and Optimize Baldeagle inside *new* QSFP board
# 
# This function configures and Optimizes Baldeagle lanes in NRZ or PAM4 modes
#
# It assumes QSFP cables connected between: Slice 0: A0-3 to A4-7 
#                                           Slice 0: B0-3 to B4-7
#                                           Slice 1: A0-3 to A4-7
#                                           Slice 1: B0-3 to B4-7
# 
#
####################################################################################################
def config_qsfp(Slice=None, nrz_lanes=[], pam4_lanes=[], nrz_data_rate=None):

    if Slice!=None: 
        mdio('connect',Slice)
        #slice_reset()
    init_qsfp(nrz_lanes, pam4_lanes)
    
    global gLane; gLane = sorted(list(set(pam4_lanes + nrz_lanes)))

    set_bandgap('on', 'all') 
    set_top_pll(pll_side='both', freq=195.3125)
    
    #if not fw_loaded():
    #    fw_load()

    # set up and opt in PAM4 or NRZ mode, if needed
    if nrz_lanes!=[]: 
        opt_lane (mode='nrz',datarate=nrz_data_rate,input_mode='ac',lane=nrz_lanes)
    if pam4_lanes!=[]: 
        opt_lane (mode='pam4',input_mode='ac',lane=pam4_lanes)
        
    
    fw_config_lane()
    #auto_pol(print_en=1)
    serdes_params(lane=gLane)
    rx_monitor(rst=1,lane=gLane)
    fw_config_lane()

####################################################################################################  
# Slice 0, B0-B3: PAM4 loopback, A0-A7: NRZ to Falcon
#
####################################################################################################
def config_ardent(Slice=0, input_mode='dc'):

    global gSlice
    # Turn off all the lanes first and later turn on only lanes under test
    mdio('connect',Slice=0)
    set_bandgap('off', 'all')
    mdio('connect',Slice=1)
    set_bandgap('off', 'all') 

    mdio('connect',Slice) # Now turn on Slice under test
    set_bandgap('on', 'all') 
    set_top_pll(pll_side='both', freq=195.3125)
    #slice_reset()

   
    ######## Set up A0-A3 in PAM4 mode
    #pam4_lanes = [0,1,2,3,4,5]
    # init_lane_pam4(input_mode=input_mode,lane=pam4_lanes) # A0-A3 in PAM4 mode
    # delta_ph(6,pam4_lanes)
    # dc_gain(1,15,1,1, pam4_lanes)
    # ffe_pol(1, 0, 1, 1,pam4_lanes)
    # ffe_taps(0x01,0x01,0x18,0x3f,0x01,0x01,pam4_lanes)
    # [skef(1,2,lane) for lane in pam4_lanes]
    # [edge(8,8,8,8,lane) for lane in pam4_lanes]
    
    #lr(pam4_lanes) # reset lanes
 
    ######## Set up B0-B7 in NRZ mode
    #nrz_lanes = [8,9,10,11,12,13,14,15]
    # init_lane_nrz(input_mode=input_mode,lane=nrz_lanes) # B0-B7 in NRZ mode
    # lr(nrz_lanes) # reset lanes
    
    #fec_config(Slice=0)

    ########################## Ardent Board's RX and TX Polarities for Slice 0 and 1
    global RxPolarityMap; RxPolarityMap=[]
    global TxPolarityMap; TxPolarityMap=[]
                                  #Slice  [A0.............A7, B0,..........B7]
    RxPolarityMap.append([]); RxPolarityMap[0]=[0,0,0,0,0,0,0,0,   1,1,1,1,1,1,1,1] # Slice 0 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[0]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 0 lanes, TX Polarity
    RxPolarityMap.append([]); RxPolarityMap[1]=[0,0,0,0,0,0,0,0,   0,0,0,0,0,0,0,0] # Slice 1 lanes, RX Polarity
    TxPolarityMap.append([]); TxPolarityMap[1]=[1,1,1,1,1,1,1,1,   1,1,1,1,1,1,1,1] # Slice 1 lanes, TX Polarity

    ######## Set up the PAM4 lanes, A0-A3
    global gPam4Lanes; gPam4Lanes = [0,1,2,3,4,5]
    init_lane_pam4(input_mode='ac',lane=gPam4Lanes)
    #opt_lane_pam4(input_mode,gPam4Lanes)
    for ln in gPam4Lanes: # Set Polarity of each lane according the pol_list
        pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
         
    ######## Set up the NRZ lanes, B0-B7
    global gNrzLanes; gNrzLanes = [8,9,10,11,12,13,14,15]
    init_lane_nrz(input_mode='dc',lane=gNrzLanes) 
    opt_lane_nrz (input_mode='dc',lane=gNrzLanes) 
    for ln in gNrzLanes: # Set Polarity of each lane according the pol_list
        pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)

    serdes_params(lane='all')


####################################################################################################
# First step inside a python shell is to establish connection 
# to Credo Serdes Eval boards+Dongle+USB Cable to PC
#
# Usage:
#  *** mdio()                 : Show current status of Credo MDIO connection
#
#  *** mdio('connect',0)      : Connect to Credo Slice 0 (on USB port 0)
#  *** mdio('connect',1)      : Connect to Credo Slice 1 (on USB port 0)
#  *** mdio('connect',0,1)    : Connect to Credo Slice 0 (on USB port 1)
#  *** mdio('connect',1,1)    : Connect to Credo Slice 1 (on USB port 1)
#
#  *** mdio('disconnect',0)   : Disconnect MDIO to Credo Slice 0 (on USB port 0)
#  *** mdio('disconnect',1)   : Disconnect MDIO to Credo Slice 1 (on USB port 0)
#  *** mdio('disconnect',0,1) : Disconnect MDIO to Credo Slice 0 (on USB port 1)
#  *** mdio('disconnect',1,1) : Disconnect MDIO to Credo Slice 1 (on USB port 1)
#
####################################################################################################
def mdio(connect=None, Slice=None, usb_port=None):
    time.sleep(0.5)
    global gUsbPort
    global gSlice
    
    SWAPPING_USB_PORT=0
    SWAPPING_SLICE=0
                    
    if (connect==None): # only checking MDIO connection status
        mdio_status = get_mdio_status()
        if (mdio_status == MDIO_DISCONNECTED): 
            print ('\n...Credo MDIO Already Disconnected from USB Port %d Slice %d'%(gUsbPort,gSlice))
        elif(mdio_status == MDIO_CONNECTED):
            print ('...Credo MDIO Already Connected to USB Port %d Slice %d'%(gUsbPort,gSlice))
        else: # mdio_status = MDIO_LOST_CONNECTION
            val0 = rreg(0x0,0)
            print ('\n***> Credo MDIO Connection is LOST on USB Port %d Slice %d'%(gUsbPort,gSlice)),
            print ('\n***> Reading back invalid value 0x%04X for Credo Registers' % val0),
            print ('\n***> Disconnecting MDIO on USB Port %d Slice %d'%(gUsbPort,gSlice))
            #chip.disconnect()
            mdio_status = MDIO_DISCONNECTED
        
    else: # asked to connect/dis-connect
        if usb_port!=None and usb_port!=gUsbPort:
            SWAPPING_USB_PORT=1
            gUsbPort=usb_port
        
        if Slice!=None and Slice!=gSlice:
            SWAPPING_SLICE=1        
            gSlice=Slice

        if connect == 1 or connect =='connect':
            mdio_status= get_mdio_status()
            if ( SWAPPING_SLICE==0 and mdio_status == MDIO_CONNECTED): # Already connected and reading valid values
                print ('...Credo MDIO Already Connected to USB Port %d Slice %d'%(gUsbPort,gSlice))
            else: # changing Slice or not connected to any Slice yet, issue connect() command
                #if (SWAPPING_SLICE and mdio_status == MDIO_CONNECTED)
                #chip.connect(phy_addr=gSlice, usb_sel=gUsbPort)
                mdio_status= get_mdio_status()
                if (mdio_status == MDIO_CONNECTED): 
                    print ('...Credo MDIO Connected to USB Port %d Slice %d'%(gUsbPort,gSlice))
                else:
                    #val0 = rregLane(0x0)
                    print ('\n***> Credo MDIO Connection Attempt Failed on USB Port %d Slice %d'%(gUsbPort,gSlice)),
                    #print ('\n***> Reading back 0x%04X! for SerDes Registers' % val0),
                    print ('\n***> Disconnecting Python from MDIO on USB Port %d Slice %d'%(gUsbPort,gSlice)),
                    print ('\n***> Fix Dongle Connection Issue and Try again!')
                    #chip.disconnect()
                    mdio_status = MDIO_DISCONNECTED
                    
        elif connect == 0 or connect =='disconnect': # connect == 0 (i.e. Disconnect MDIO)
            mdio_status = get_mdio_status()
            #if (mdio_status != MDIO_CONNECTED):
                #chip.connect(phy_addr=gSlice, usb_sel=gUsbPort)
            #chip.disconnect()
            print ('\n...Credo MDIO Disconnected from USB Port %d Slice %d'%(gUsbPort,gSlice))
            mdio_status = MDIO_DISCONNECTED
    time.sleep(0.5)
####################################################################################################
# Select Slice/slicer inside the part to work with.
# Sets Global variable "gSlice" to the selected Slice
#
# Usage:
#  *** sel_slice()   : Show current Slice conencted to. returns 0: Slice 0, 1: Slice 1
#  *** sel_slice(0)  : Connect to Slice/slicer 0 
#  *** sel_slice(1)  : Connect to Slice/slicer 1
####################################################################################################
def sel_slice(slice=None): # options: None, 0 or 1
    global gSlice
                        
    if (slice!=None): # asked to connect to a particular Slice inside the same chip, sharing MDIO bus
        #chip.connect(phy_addr=slice, usb_sel=gUsbPort)
        gSlice=slice
                    
    return gSlice

####################################################################################################
def get_mdio_status():
  mdio_status = MDIO_LOST_CONNECTION
  val0 = rreg(0x0,0)
  val1 = rreg(0x1,0)
  
  if ((val0==0xffff or val0==0x7fff) and val1==val0):
    mdio_status = MDIO_LOST_CONNECTION
    
  elif ((val0==0x0001) and val1==val0):
    mdio_status = MDIO_DISCONNECTED
    
  elif (val0!=0x0000 and val0!=0x0001 and val0!=0xffff and val1!=val0): # Already connected and reading valid values
      mdio_status = MDIO_CONNECTED
     
  return mdio_status

####################################################################################################
# 
# Process the specific lane(s) passed 
# and return an integer list of "lanes" to the calling function to loop through list of lanes
#
####################################################################################################
def get_slice_list(slice=None):
    if gDualSliceMode==True:
        slices=[0,1]
    else:
        if slice==None:          slices=[gSlice] # single slice download mode on the Slice already selected
        elif type(slice)== int:  slices=[slice]     # single slice download mode 
        elif type(slice)== list: slices=slice       # single slice download mode 
        else:                    slices=[0,1]    # broadcast download mode to both slices
    
    return slices
####################################################################################################
# 
# Process the specific lane(s) passed 
# and return an integer list of "lanes" to the calling function to loop through list of lanes
#
####################################################################################################
def get_lane_list(lane=None):
    if lane==None:            lane=gLane
    if type(lane)==int:       lanes=[lane]
    elif type(lane)==list:    lanes=lane
    elif type(lane)==str and lane.upper()=='ALL': 
        lanes=range(len(lane_name_list))
    lanes.sort()
    return lanes
####################################################################################################
# 
# Processes the lane list passed
# Based on number of A-lanes in the list returns...
# ... all lanes connected from A side to B side in retimer mode
#
####################################################################################################
def get_retimer_lane_list(lane=None, cross_mode=False):

    lanes=get_lane_list(lane)
    A_lanes=[x for x in lanes if x < 8]
    lanes=A_lanes[:]
    for ln in A_lanes:
        if cross_mode==False:
            lanes.append(ln+8)  # retimer straight mode lanes A0-A7 to B0-B7
        else:
            if ln<4:  
                lanes.append(ln+12) # retimer cross mode lanes A0-A3 to B4-B7
            elif ln>=4: 
                lanes.append(ln+4)  # retimer cross mode lanes A4-A7 to B0-B3
    lanes.sort()            
    return lanes
  
####################################################################################################
# register write/read
# intended for engineering mode use only
# specifically made for use in python shell command
# and not be called from other functions
####################################################################################################
def reg(addr, val=None, lane=None,slice=None):

    full_addr=False
    lanes  = get_lane_list(lane)
    slices = get_slice_list(slice)

    if   type(addr)==int:  addr_list=[addr]
    elif type(addr)==list: addr_list=addr

    if addr_list[0]>= 0x200: # >0x200 taken as not per-lane registers
        full_addr=True
        lanes=[0]
    
    ##### Print Headers
    print("\nSlice ADDR:"),
    #print("\n ADDR:"),
    if full_addr: 
        print("VALUE"),
    else:
        for ln in lanes:
            print("  %s" %(lane_name_list[ln])),
   
    ##### Write registers first, if value given
    if val!=None:
        for slc in slices:
            sel_slice(slc)
            for add in addr_list:
                for ln in lanes:
                    wreg(add,val,ln)
    ##### Read registers, one address per row
    for add in addr_list:
        for slc in slices:
            sel_slice(slc)
            if full_addr: print("\n   S%d %04X:" % (gSlice,add) ),
            else:         print("\n   S%d  %03X:"% (gSlice,add) ),
            for ln in lanes:
                print("%04X" % (rreg(add,ln)) ),
            
####################################################################################################
def rreg(addr, lane = None):
    '''
    read from a register address, register offset or register field
    '''
    if lane==None: lane=gLane
    
    #lane_mode_list
    if type(addr) == int:
        if (addr & 0xf000) == 0: addr += lane_offset[lane_name_list[lane]]
        return MdioRd(addr)
    elif type(addr) == list:
        val = 0
        i = 0
        while(i  < (len(addr)-1)):
            addr_1 = addr[i]
            if (addr[i] & 0xf000) == 0: addr_1 += lane_offset[lane_name_list[lane]]            
            val_tmp = MdioRd(addr_1)
            i += 1
            mask = sum([1<<bit for bit in range(addr[i][0], addr[i][-1]-1, -1)])
            val_tmp = (val_tmp & mask)>>addr[i][-1]
            val = (val<<(addr[i][0]-addr[i][-1]+1)) + val_tmp
            i += 1
        return val
    else:
        print("\n***Error reading register***")
        return -1
        
####################################################################################################
def wreg(addr, val, lane = None):
    '''
    write to a register address, register offset or register field
    '''
    #global chip
    if lane==None: lane=gLane[0]
    if type(addr) == int:
        if (addr & 0xf000) == 0: addr += lane_offset[lane_name_list[lane]]
        MdioWr(addr, val)
    elif type(addr) == list:
        addr_1 = addr[0]
        if (addr[0] & 0xf000) == 0: addr_1 += lane_offset[lane_name_list[lane]]
        val_old = MdioRd(addr_1)
        mask = sum([1<<bit for bit in range(addr[1][0], addr[1][-1]-1, -1)])
        val_new = (val_old & ~mask) + (val<<addr[1][-1] & mask)
        MdioWr(addr_1, val_new)
    else:
        print("\n***Error writing register***")
     
        
####################################################################################################
def rreg_new(addr, lane=None):
    '''
    read from a register address, register offset or register field
    '''
    lanes = get_lane_list(lane)
    for ln in lanes:   
        if type(addr) == int:
            addr_l = addr
            if (addr_l & 0xf000) == 0: addr_l = addr + lane_offset[lane_name_list[ln]]
            return MdioRd(addr_l)
        elif type(addr) == list:
            val = 0
            i = 0
            while(i  < (len(addr)-1)):
                addr_ll = addr[i]
                if (addr_ll & 0xf000) == 0: addr_ll = addr_ll + lane_offset[lane_name_list[ln]]            
                val_tmp = MdioRd(addr_ll)
                i += 1
                mask = sum([1<<bit for bit in range(addr[i][0], addr[i][-1]-1, -1)])
                val_tmp = (val_tmp & mask)>>addr[i][-1]
                val = (val<<(addr[i][0]-addr[i][-1]+1)) + val_tmp
                i += 1
            return val
        else:
            print("\n***Error reading register***")
            return -1
        
####################################################################################################
def wreg_new(addr, val, lane=None):
    '''
    write to a register address, register offset or register field
    '''
    lanes = get_lane_list(lane)
    
    for ln in lanes:   
        if type(addr) == int:
            addr_l = addr
            if (addr & 0xf000) == 0: addr_l = addr + lane_offset[lane_name_list[ln]]
            MdioWr(addr_l, val)
        elif type(addr) == list:
            addr_ll = addr[0]
            if (addr_ll & 0xf000) == 0: addr_ll = addr_ll + lane_offset[lane_name_list[ln]]
            val_old = MdioRd(addr_ll)
            mask = sum([1<<bit for bit in range(addr[1][0], addr[1][-1]-1, -1)])
            val_new = (val_old & ~mask) + (val<<addr[1][-1] & mask)
            MdioWr(addr_ll, val_new)
        else:
            print("\n***Error writing register***")
     
####################################################################################################
def wregBits(addr, bits, val, lane = None):
    '''
    write to a register field
    '''

    addr = [addr, bits]    
    wreg(addr, val, lane)
    
####################################################################################################
def wregMask(addr, mask, val, lane = None):
    '''
    write multiple bit fields, as defined by mask, to a register address
    '''
    if lane==None: lane=gLane[0]
    value_old = rreg(addr)
    value_new = (value_old & ~mask) | (value & mask)
    wreg(addr, value)

####################################################################################################
def rregBits(addr, bits, lane = None):
    '''
    read from a register field
    '''
    if lane==None: lane=gLane[0]
    addr = [addr, bits]
    return rreg(addr, lane)

####################################################################################################
# Similar to the function in main script
# Added lane_reset at the end 
####################################################################################################
def load_setup(filename = None):

    global gSetupFileName
    if filename==None:
        filename=gSetupFileName # Use the same filename used last time save_setup() was called.

    try:
        f=open(filename, 'r')
        script = f.read()
        f.close()
    except IOError:
        print ("\n***Error: Can't Find Register Setup File: '%s' <<<\n" %filename)
        return
    insts = re.findall("^[\da-fA-F]{4}[     ]+[\da-fA-F]{4}", script, flags = re.MULTILINE)
    insts = map(lambda t: (int(t[0:4], 16), int(t[5:], 16)), insts) # convert extracted texts to 16-bit integers addr and data
    for inst in insts:
        MdioWr(inst[0], inst[1])
        
    #for lane in lane_name_list:
    #   lane_reset(lane)
    
    print ("...Loaded Slice %d Registers from File: %s" %(gSlice,filename))
     
####################################################################################################
# Similar to the function in main script
# changed the order of addresses saved to be sequential from 0x7000 to 0x8FFF 
####################################################################################################
def save_setup(filename = None, lane=None):
    
    global gSetupFileName
    global gChipName

    if lane==None: 
        lanes = range(0,len(lane_name_list)) # Save all lanes' settings by default
    else:
        lanes = get_lane_list(lane)
  
    try:
        f=open(filename, 'r')
        script = f.read()
        f.close()
        print ("\n*** A file with the same name already exists. Choose another file name!  <<<<"),
        return
    except IOError:
        print (" ")

    if len(lanes)==1: 
        lanes_str='_Device%d_Lane%d_%s_' %(gSlice,lanes[0],gEncodingMode[gSlice][lanes[0]][0].upper())
    else:
        lanes_str='_Device%d_Lanes%d-%d_%s_' %(gSlice,lanes[0],lanes[-1],gEncodingMode[gSlice][lanes[0]][0].upper())

    timestr = time.strftime("%Y%m%d_%H%M%S")
    if filename == None:
        filename = gChipName + lanes_str + timestr + '.txt'
    gSetupFileName=filename
    log = open(filename, 'w')
    log.write('\n#---------------------------------------------'),
    log.write('\n# File: %s' % filename),
    log.write("\n# %s Rev %2.1f Registers" %(gChipName, chip_rev())),
    log.write("\n# %s" % time.asctime())
    log.write('\n#---------------------------------------------\n'),
    log.close()
    
    ##### Save FW SERDES PARAMS
    log_file = open(filename, 'a+')
    temp_stdout = sys.stdout
    sys.stdout = log_file
    serdes_params(lanes)
    sys.stdout = temp_stdout
    log_file.close()

    if lanes==range(0,len(lane_name_list)): # First, save Top PLL Settings only if all lanes are going to be saved
        reg_group_dump(0x9500, range(0x00, 0x16, 1), 'PLL Fsyn1 registers', filename)
        reg_group_dump(0x9600, range(0x00, 0x16, 1), 'PLL Fsyn0 registers', filename)
    for lane in lanes:
        reg_group_dump(lane_offset[lane_name_list[lane]], range(0x00, 0x1ff+1, 1), 'per lane registers', filename)
        
    reg_group_dump(0x3000, range(0x000,0x3FF, 1), 'Link Training Registers', filename)
    reg_group_dump(0x4000, range(0x000,0xBFF, 1), 'FEC Registers A', filename)
    reg_group_dump(0x5000, range(0x000,0xBFF, 1), 'FEC Registers B', filename)
    reg_group_dump(0x9800, range(0x000,0x0DA, 1), 'Top Registers', filename)
    reg_group_dump(0xB000, range(0x000,0x00F, 1), 'FEC Analyzer Registers', filename)
    reg_group_dump(0xB000, range(0x03F,0x0FF, 1), 'TSensor, VSensor Registers', filename)
    
    ##### Save FW Register values
    log_file = open(filename, 'a+')
    temp_stdout = sys.stdout
    sys.stdout = log_file
    fw_reg()
    sys.stdout = temp_stdout
    log_file.close()

    lanes_str='Slice%d, Lanes %d-%d,' %(gSlice,lanes[0],lanes[-1])
    print ("...Saved Slice %s Registers to Setup File: %s" %(lanes_str,filename))


####################################################################################################
# Similar to the function in main script
# changed the order of addresses saved to be sequential from 0x7000 to 0x8FFF 
####################################################################################################   
def reg_group_dump(base_addr, addr_range, addr_name, filename):
    log = open(filename, "a+")
    log.write('\n\n#---------------------------------------------')
    log.write('\n#%s (R%04X to R%04X)' % (addr_name, base_addr + addr_range[0], base_addr + addr_range[-1]) )
    log.write('\n#Addr Value')
    #print('\n%s (R%04X to R%04X)' % (addr_name, base_addr + addr_range[0], base_addr + addr_range[-1]) ),
    log.write('\n#---------------------------------------------\n'),

    for i in addr_range:
        #print("\nR%04X: " % (base_addr + i)),
        #log.write("\n%04X " % (base_addr + i)),
        for j in range(1):
            addr = base_addr + j * 0x100 + i
            val = MdioRd(addr)
            #print(" %04X" % val)
            if addr == 0x9501 or addr == 0x9601:
                val_01 = val
                val = val & 0xfffb
            if addr == 0x9512 or addr == 0x9612:
                val_12 = val
                val = val & 0x7fff
            log.write("%04X %04X\n" % (addr, val))
    if base_addr == 0x9500 or base_addr == 0x9600:
        log.write("%04X %04X # TOP PLL POR TX [15] toggle\n" % (base_addr+0x12, val_12))
    if base_addr == 0x9500 or base_addr == 0x9600:
        log.write("%04X %04X # TOP PLL PU [2] toggle\n" % (base_addr+0x01, val_01))
    log.write("\n")
    log.close()
####################################################################################################
# 
# prints or saves values for same address of each lane in one row, for quick comparison and debugging.
# Example:
#--------------------------------------------------------------------------------------------------
#   ,   A0,   A1,   A2,   A3,   A4,   A5,   A6,   A7,   B0,   B1,   B2,   B3,   B4,   B5,   B6,   B7
#Addr,7000 ,7200 ,7400 ,7600 ,7800 ,7A00 ,7C00 ,7E00 ,8000 ,8200 ,8400 ,8600 ,8800 ,8A00 ,8C00 ,8E00 
#--------------------------------------------------------------------------------------------------
#000 ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B 
#001 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 
#
# Note, this is a log file for debug only and not suitable to be called by "load_setup()"
# 
# reg_group_dump_debug_mode(range(0x7000,0x8F00+1,0x200), range(0,0x1FF+1, 1), "SerDes Lane Registers")
# reg_group_dump_debug_mode(range(0x3000,0x3F00+1,0x100), range(0,0x063+1, 1), "Link Training Registers")
# reg_group_dump_debug_mode(range(0x9500,0x9600+1,0x100), range(0,0x015+1, 1), "Top PLL Registers")
####################################################################################################   
def reg_group_dump_debug_mode(base_addr_range, lane_addr_range, addr_name, filename=None):
    
    if filename!=None:
        log_file = open(filename, 'a+')
        temp_stdout = sys.stdout
        sys.stdout = log_file

    separator='\n-----'
    for lane_base_addr in base_addr_range:
        separator += '------'
        
    ### header
    print('\n %s   (Addr: 0x%04X to 0x%04X)' % (addr_name, base_addr_range[0], base_addr_range[-1]+lane_addr_range[-1]) ),
    print('%s'%separator),
    print("\n     "),
    for lane in lane_name_list:
        print("%4s " % (lane)),   
    print("\nAddr:"),
    for lane_base_addr in base_addr_range:
        print("%04X " % (lane_base_addr)),
    print('%s'%separator),
    
    ### Data
    for pre_lane_addr in lane_addr_range:
        print("\n %03X:" % (pre_lane_addr)),
        prev_val = 0xeeeee
        for base_addr in base_addr_range:
            val = MdioRd (base_addr + pre_lane_addr)
            diff= '<' if (base_addr!=base_addr_range[0] and val!=prev_val) else ' '
            print("%04X%s" % (val,diff)),
            prev_val = val
            
    print('%s'%separator),
    print("\n"),
    
    if filename!=None:
        sys.stdout = temp_stdout
        log_file.close()
        #print ("\n...Saved Registers as Debug Setup File: %s" %filename)
####################################################################################################
# 
# saves the same address values for all lanes in one row, for quick comparison and debugging.
# Example:
#--------------------------------------------------------------------------------------------------
#   ,   A0,   A1,   A2,   A3,   A4,   A5,   A6,   A7,   B0,   B1,   B2,   B3,   B4,   B5,   B6,   B7
#Addr,7000 ,7200 ,7400 ,7600 ,7800 ,7A00 ,7C00 ,7E00 ,8000 ,8200 ,8400 ,8600 ,8800 ,8A00 ,8C00 ,8E00 
#--------------------------------------------------------------------------------------------------
#000 ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B ,106B 
#001 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 ,0800 
#
# Note, this is a log file for debug only and not suitable to be called by "load_setup()"
#
####################################################################################################   
def save_setup_debug_mode(filename = None):
    '''
    saves the same address values for all lanes in one row, for quick comparison and debugging.
    '''
    global gChipName

    timestr = time.strftime("%Y%m%d_%H%M%S")
    if filename == None:
        filename = gChipName + '_Reg_Setup_Debug_Mode' + timestr + '.txt'
    log = open(filename, 'w')
    log.write('\n----------------------------------------------------------------------'),
    log.write('\n File: %s' % filename),
    log.write('\n %s Rev %2.1f' % (gChipName, chip_rev(print_en=0))),
    log.write("\n %s" % time.asctime())
    log.write('\n----------------------------------------------------------------------\n\n'),
    log.close()
    
    reg_group_dump_debug_mode(range(0x7000,0x8F00+1,0x200), range(0,0x1FF+1, 1), "SerDes Lane Registers", filename )
    reg_group_dump_debug_mode(range(0x3000,0x3F00+1,0x100), range(0,0x063+1, 1), "Link Training Registers", filename)
    reg_group_dump_debug_mode(range(0x9500,0x9600+1,0x100), range(0,0x015+1, 1), "Top PLL Registers", filename)
    
    ##### Save FW Serdes Params and FW Regiters
    log_file = open(filename, 'a+')
    temp_stdout = sys.stdout
    sys.stdout = log_file
    serdes_params(lane=range(16)) #####
    fw_reg()                      #####
    sys.stdout = temp_stdout
    log_file.close()

    print ("\n...Saved Registers as Debug Setup File: %s" %filename)

####################################################################################################
# 
# Reset Slice completely (ready to be initialized next)
####################################################################################################
def slice_reset(t=0.010):

    #global gSlice
    #chip.connect(phy_addr=Slice); gSlice=Slice
    get_lane_mode('all') # update the Encoding modes of all lanes for this Slice
    soft_reset()
    time.sleep(0.1)
    set_bandgap('on', 'all') 
    set_top_pll(pll_side='both', freq=195.3125)
    all_lanes=range(len(lane_name_list))
    
    # put lanes in PAM4 mode and do a logic reset
    set_lane_mode(mode='pam4',lane=all_lanes)
    time.sleep(0.1)
    logic_reset()
    time.sleep(0.1)    
    
    # put lanes in NRZ mode and do a logic reset
    set_lane_mode(mode='nrz',lane=all_lanes)
    time.sleep(0.1)
    logic_reset()
    time.sleep(0.1)
    
    for ln in range(16):
        ####disable AN/LT registers as default
        wreg(0x3000+(0x100*ln),0x0000,lane=0)
        wreg(0xe000+(0x100*ln),0xc000,lane=0)
        wreg([0x041,[15]], 0,ln) # Toggle PAM4_RX_ENABLE
        wreg([0x041,[15]], 1,ln) # Toggle PAM4_RX_ENABLE
        time.sleep(t)
        wreg([0x041,[15]], 0,ln) # Toggle PAM4_RX_ENABLE
        wreg([0x0b0,[11]], 0,ln) # Toggle NRZ_PRBS_ENABLE
        wreg([0x0b0,[11]], 1,ln) # Toggle NRZ_PRBS_ENABLE
        time.sleep(t)
        wreg([0x0b0,[11]], 0,ln) # Toggle NRZ_PRBS_ENABLE

    fec_reset()
    
   
    #set_bandgap('off', 'all')
    #chip.disconnect()
 
    print("\n...Slice %d is FULLY reset!" % gSlice),

####################################################################################################
def soft_reset():
    wreg(Pam4Reg.chip_rst_addr, Pam4Reg.chip_soft_rst_val)
    wreg(Pam4Reg.chip_rst_addr, 0x0)

def logic_reset():
    wreg(Pam4Reg.chip_rst_addr, Pam4Reg.chip_logic_rst_val)
    wreg(Pam4Reg.chip_rst_addr, 0x0)

def cpu_reset():
    wreg(Pam4Reg.chip_rst_addr, Pam4Reg.chip_cpu_rst_val) 
    wreg(Pam4Reg.chip_rst_addr, 0x0)

def reg_reset():
    wreg(Pam4Reg.chip_rst_addr, Pam4Reg.chip_reg_rst_val) 
    wreg(Pam4Reg.chip_rst_addr, 0x0)
    
####################################################################################################
# FW_LOAD_FILE_INIT()
#
# Downloads FW to one slice or both slices of a chip at the same time
#
# usage:
#       fw_load('1.19.00')           # load FW ver 1.19.00 to slice already selected
#       fw_load('1.19.00', sl=[0])   # load FW ver 1.19.00 to slice 0
#       fw_load('1.19.00', sl=[1])   # load FW ver 1.19.00 to slice 1
#       fw_load('1.19.00', sl=[0,1]) # load FW ver 1.19.00 to both slices in parallel
####################################################################################################
def fw_load_file_init (file_name=None, path_name=None):

    global gFwFileName
    global gFwFileNameLastLoaded
    
    #### Define FW file folder and the filename according to chip revision
    file_path = '/usr/share/DosFirmwareImages/'
    file_name_prefix_rev_1 = 'BE.fw.'
    file_name_prefix_rev_2 = 'BE2.fw.'
    file_name_extension = '.bin'    
    if file_name==None:
        try:
            gFwFileName
        except NameError:
            print("\n*** FW Not loaded. No FW file name is defined ***\n\n")
            return False
        else:
            fw_file_name=gFwFileName
    else:
        if not (file_name_extension in file_name) and not ('.fw.' in file_name): 
            file_name_prefix = file_name_prefix_rev_2 if chip_rev()==2.0 else file_name_prefix_rev_1
            fw_file_name = file_name_prefix + file_name + file_name_extension
        elif not (file_name_extension in file_name) and ('.fw' in file_name): 
            fw_file_name = file_name + file_name_extension
        else:
            fw_file_name = file_name # use the filename exactly as passed to function
    if path_name !=None:    
        fw_file_name = file_path + fw_file_name
    gFwFileNameLastLoaded = fw_file_name # update this. Used by fw_info()
    
    if not os.path.exists(fw_file_name):
        print("\n...FW LOAD ERROR: Error Opening FW File: %s\n" %(fw_file_name))      
        return False
    
    return  fw_file_name         
####################################################################################################
def fw_load_init ():
    ##### Before download, make sure Top PLL is set, cal'ed and FW is Unloaded in each slice
    #for slice_num in slices:
    #sel_slice(slice_num)
    set_top_pll(pll_side='both') # Setup Top PLLs first
    pll_cal_top()                # Calibrate Top PLLs 
    fw_unload(print_en=0)        # Unload FW from each slice
           
####################################################################################################
def fw_load_broadcast_mode (mode='on'):
    
    FW3 = 0x985A # Broadcast-Mode Download control register

    if mode.upper() =='ON':
        wreg(FW3,0x8888)
    else:
        wreg(FW3,0x0000)
           
####################################################################################################
def fw_load_main (fw_file_name=None,broadcast_mode='OFF',wait=0.001):
    print_en=0

    ##### FW Download Register addresses
    FW0 = 0x9F00
    FW1 = 0x9802
    FW2 = 0x9805
    
    ##### FW Download starts here
    fw_file_ptr = open(fw_file_name, 'rb')
    fw_data = fw_file_ptr.read()
    start = 4096
    file_hash_code = struct.unpack_from('>I', fw_data[start   :start+4 ])[0]
    file_crc_code  = struct.unpack_from('>H', fw_data[start+4 :start+6 ])[0]
    file_date_code = struct.unpack_from('>H', fw_data[start+6 :start+8 ])[0]
    entryPoint     = struct.unpack_from('>I', fw_data[start+8 :start+12])[0]
    length         = struct.unpack_from('>I', fw_data[start+12:start+16])[0]
    ramAddr        = struct.unpack_from('>I', fw_data[start+16:start+20])[0]
    data           = fw_data[start+20:]

    d=datetime.date(1970, 1, 1) + datetime.timedelta(file_date_code)

    if print_en: print "fw_load Hash Code : 0x%06x" % file_hash_code
    if print_en: print "fw_load Date Code : 0x%02x (%04d-%02d-%02d)" % (file_date_code, d.year, d.month, d.day)
    if print_en: print "fw_load  CRC Code : 0x%04x" % file_crc_code
    if print_en: print "fw_load    Length : %d" % length
    if print_en: print "fw_load     Entry : 0x%08x" % entryPoint
    if print_en: print "fw_load       RAM : 0x%08x" % ramAddr

    dataPtr = 0
    sections = (length+23)/24

    ##### FW Unload
    wreg(FW2, 0xFFF0)
    wreg(FW1, 0x0AAA)
    wreg(FW1, 0x0000)    
    if broadcast_mode.upper()=='ON': # broadcast download mode, use fixed delay
        time.sleep(.01)      
    else:                # single-slice download mode, status read back
        start_time=time.time()
        checkTime = 0
        status = rreg(FW2)
        while status != 0:
            status = rreg(FW2)
            checkTime += 1
            if checkTime > 100000:
                print '\n...FW LOAD ERROR: : Wait for FW2=0 Timed Out! FW2 = 0x%X'% status, #Wait for FW2=0: 0.000432 sec
                break
        stop_time=time.time()    
    wreg(FW2, 0x0000)
    ##### FW Unload Done
    
    download_start_time=time.time()

    ##### Main FW Download loop
    i = 0
    while i < sections:
        checkSum = 0x800c
        wreg(FW0+12, ramAddr>>16)
        wreg(FW0+13, (ramAddr & 0xFFFF))
        checkSum += ( ramAddr >> 16 ) + ( ramAddr & 0xFFFF )
        for j in range( 12 ):                                               
            if (dataPtr > length):
                mdioData = 0x0000
            else: 
                mdioData = struct.unpack_from('>H', data[dataPtr:dataPtr+2])[0]
            wreg(FW0+j, mdioData)
            checkSum += mdioData
            dataPtr += 2
            ramAddr += 2
 
        wreg(FW0 + 14, (~checkSum+1) & 0xFFFF)
        wreg(FW0 + 15, 0x800C)
        #print '\nDEBUG fw_load: section %d, Checksum %X' % (i, checkSum),
 
        
        if broadcast_mode.upper()=='ON': # broadcast download mode, use fixed delay passed to the function
            time.sleep(wait)      
        else:                # single-slice download mode, status read back per section
            checkTime = 0
            status = rreg(FW0 + 15) 
            while status == 0x800c:
                status = rreg(FW0 + 15)
                checkTime += 1
                if checkTime > 1000:
                    print '\n...FW LOAD ERROR: Write to Ram Timed Out! FW0 = %x'% status,
                    break
                    
            if checkTime>0:
                if print_en: print ('\nfw_load: section %d, CheckTime= %d' % (i, checkTime)),

            if status != 0:
                print("\n...FW LOAD ERROR: Invalid Write to RAM, section %d, Status %d"%(i,status)),         
        i += 1
        print("\b\b\b\b\b%3.0f%%"%(100.0 *i/sections)), # Display % download progressed
    ##### End of Main FW Download loop
    
    ##### Last steps of FW Download
    wreg(FW0 + 12, entryPoint>>16)
    wreg(FW0 + 13, ( entryPoint & 0xFFFF ))
    checkSum = ( entryPoint >> 16 ) + ( entryPoint & 0xFFFF ) + 0x4000
    wreg(FW0 + 14, ( ~checkSum+1 ) & 0xFFFF)
    wreg(FW0 + 15, 0x4000)    
    time.sleep(.1)
    
    download_time = time.time()-download_start_time     
    fw_file_ptr.close()
    
    return download_time, file_crc_code
####################################################################################################
def fw_load_status (file_crc_code, lc=0,ph=0,sl=0):
    ##### Check FW CRC for each slice and confirm a good download
    fw_info(lc,ph,sl)
    crc_code = fw_crc (print_en=0)
    if crc_code != file_crc_code:
        print("\n\n...Slice %d FW LOAD ERROR: CRC Code Not Matching, Expected CRC: 0x%04x -- Actual CRC: 0x%04x\n\n" %(sl,file_crc_code,crc_code))
        return False
    else:
        return True
    #return hash_code, crc_code, date_code

####################################################################################################
# FW_LOAD()
#
# Downloads FW to one slice or both slices of a chip at the same time
#
# usage:
#       fw_load('1.19.00')           # load FW ver 1.19.00 to slice already selected
#       fw_load('1.19.00', sl=[0])   # load FW ver 1.19.00 to slice 0
#       fw_load('1.19.00', sl=[1])   # load FW ver 1.19.00 to slice 1
#       fw_load('1.19.00', sl=[0,1]) # load FW ver 1.19.00 to both slices in parallel
####################################################################################################
def fw_load (file_name=None,path_name=None,lc=0,ph=0,sl=None, wait=0.0001):

    slices = get_slice_list(sl)

    if len(slices) > 1:
        broadcast_mode='ON'
        print("...Broadcast-Mode Downloading FW to both slices...   "),
    else:
        broadcast_mode='OFF'
        print("...Downloading FW to slice %d...    "%slices[0]),
        
    #### 1. Define FW file folder and the filename according to chip revision
    fw_file_name = fw_load_file_init(file_name,path_name)
    if fw_file_name == False:
        return 
        
    ##### 2. Before download, make sure Top PLL is set, cal'ed and FW is Unloaded in each slice
    for slice_num in slices:
        sel_slice(slice_num) 
        fw_load_init()
    
    ##### 3. if broadcast FW download  mode, enable it in each slice first before moving forward
    for slice_num in slices:
        sel_slice(slice_num) 
        fw_load_broadcast_mode(broadcast_mode)
        
    ##### 4. FW Download starts here
    sel_slice(slices[0])
    download_time, expected_crc_code = fw_load_main(fw_file_name,broadcast_mode,wait)
    
    ##### 5. if in broadcast FW download mode mode, disable it right away in each slice
    for slice_num in slices:
        sel_slice(slice_num) 
        fw_load_broadcast_mode('OFF')
    
    print(' (%2.2f sec)'% (download_time))
    #print("\r"),

    ##### 6. Check FW CRC for each slice and confirm a good download
    for slice_num in slices:
        sel_slice(slice_num) 
        if fw_load_status(expected_crc_code,lc,ph,slice_num) == False:
            sel_slice(slices[0]) # select the first slice before exiting
            return False
            
       
    sel_slice(slices[0]) # select the first slice before exiting
    
####################################################################################################
# This function checks real time for ALL FW related parameters 
# 
####################################################################################################
def fw_info(lc=0,ph=0,sl=None,print_en=True):
    global gFwFileName
    global gFwFileNameLastLoaded
    try:
        gFwFileNameLastLoaded
    except NameError:
        fw_bin_filename = 'FW_FILENAME_UNKNOWN'
    else:
        fw_bin_filename = gFwFileNameLastLoaded
        
    if sl==None: # if slice number is not given use gSlice
        sl=gSlice
        
    magic_code = fw_magic(print_en=0) # Expected FW magic word = 0x6A6A
    ver_code   = fw_ver(print_en=0)     # 
    hash_code  = fw_hash(print_en=0)
    crc_code   = fw_crc(print_en=0)
    date_code  = fw_date(print_en=0)
    d=datetime.date(1970, 1, 1) + datetime.timedelta(date_code)
    
    ver_str    = "VER_%02d.%02d.%02d" %(ver_code>>16&0xFF,ver_code>>8&0xFF,ver_code&0xFF)
    date_str   = "DATE_%04d%02d%02d"%(d.year, d.month, d.day)
    hash_str   = "HASH_0x%06X" %(hash_code)
    crc_str    = "CRC_0x%04X" %(crc_code)
    magic_str  = "MAGIC_0x%04X" %(magic_code)
    
    if print_en:
        #print("\n...Credo FW Info: Slice %d" %(gSlice)),
        print("...Slice (%d,%d,%d) FW Info:"%(lc,ph,sl)),
        print("'%s'," %(fw_bin_filename)),
        print("%s," %(ver_str)),
        print "%s," % (date_str),
        print("%s," %(hash_str)),
        print("%s," %(crc_str)) ,
        print("%s" %(magic_str)),
        print("")
    else:
        return fw_bin_filename, ver_str, date_str, hash_str, crc_str,  magic_str
####################################################################################################
# This function reads MAGIC WORD if a valid FW already loaded in Serdes
# 
####################################################################################################
def fw_magic(print_en=1):
    magic_word = rreg(Pam4Reg.fw_load_magic_word_addr) # reading back FW MAGIC Code
    if (magic_word == Pam4Reg.fw_load_magic_word):
        if print_en==1: print("...FW Magic Word : 0x%04X (Download Successful)\n" % magic_word),
        return magic_word
    else:
        if print_en==1: print("...FW Magic Word : 0x%04X (INVALID!!! Expected: 0x%04X)\n" % (magic_word, Pam4Reg.fw_load_magic_word)),
        return 0
    return magic_word
####################################################################################################
# This function reads SerDes Chips Revision version number for the FW already loaded in Serdes
# 
# This routine check for Chip Revsion ID using know register and value
# If ID in efuse is supported, it also read efuse ID 
# 
# Returns: 1.0 for rev A0
#          1.1 for rev A1
#          1.2 for rev A2
#          2.0 for rev B0
#          2.1 for rev B1
#          3.0 for rev C0
#
####################################################################################################
def chip_rev(print_en=0):
    rev_id_addr = [0x91,[15,0]]
  
    rev_id_reg_val = rreg(rev_id_addr,lane=8) # readback chip rev regsiter 
    
    if rev_id_reg_val == 0x0100:      
        rev_id = 2.0
        if print_en==1: print("...Credo Silicon Revision : %3.1f" %(rev_id))
    elif rev_id_reg_val == 0x0000:      
        rev_id = 1.0
        if print_en==1: print("...Credo Silicon Revision : %3.1f" %(rev_id))
    else: 
        rev_id = -1.0
        if print_en==1: print("...Credo Silicon Revision Invalid ==> (%04X)\n" %(rev_id_reg_val)),
        
    return rev_id
####################################################################################################
# This function reads FW version number for the FW already loaded in Serdes
# 
# This routine check for FW version support.
# Earlier FW did not support version command (0xF003) and return the same 
# value for date_code and ver_code. 
# 
####################################################################################################
def fw_ver(print_en=1):
    date_code = fw_date(print_en=0) # readback date code (cmd = 0xF002). 
    
    wreg(Pam4Reg.fw_ver_word_lo_addr, 0x0000) # clear readback-low value first before gets updated by next cmd
    wreg(Pam4Reg.fw_ver_word_hi_addr, 0x0000) # clear readback-high value first before gets updated by next cmd
    wreg(Pam4Reg.fw_ver_read_en_addr, Pam4Reg.fw_ver_read_en) # enable reading back FW VERSION code
    cnt=0
    while(rreg(Pam4Reg.fw_ver_read_en_addr)& Pam4Reg.fw_ver_read_status != Pam4Reg.fw_ver_read_status):
        cnt+=1
        if (cnt > 100):    break
        pass
    high_word = rreg(Pam4Reg.fw_ver_word_hi_addr) # upper byte
    low_word  = rreg(Pam4Reg.fw_ver_word_lo_addr) # lower word
    
    if date_code == low_word:      # if this is an older FW version that didn't support version command 0xF003. put month/day of date as version number  
        d=datetime.date(1970, 1, 1) + datetime.timedelta(date_code)
        ver_code = ((d.month)<<8) + d.day    
        if print_en==1: print("\n...FW Version : %02d.%02d.%02d\n" %(ver_code>>16&0xFF,ver_code>>8&0xFF,ver_code&0xFF))
    else: # if this is a FW that support version command 0xF003. get the actual FW version
        if print_en==1: print("\n...FW Version : %02d.%02d.%02d\n" %(ver_code>>16&0xFF,ver_code>>8&0xFF,ver_code&0xFF))
        ver_code = (high_word <<16) + low_word
    
    return ver_code
####################################################################################################
# This function reads HASH CODE for the FW already loaded in Serdes
# 
####################################################################################################
def fw_hash(print_en=1):
    wreg(Pam4Reg.fw_hash_word_lo_addr, 0x0000) # clear readback value first before gets updated by next cmd
    wreg(Pam4Reg.fw_hash_read_en_addr, Pam4Reg.fw_hash_read_en) # enable reading back FW HASH code
    cnt=0
    while(rreg(Pam4Reg.fw_hash_read_en_addr)& Pam4Reg.fw_hash_read_status != Pam4Reg.fw_hash_read_status):
        cnt+=1
        if (cnt > 100):    break
        pass
    high_word = rreg(Pam4Reg.fw_hash_word_hi_addr) # upper byte
    low_word  = rreg(Pam4Reg.fw_hash_word_lo_addr) # lower word
    hash_code = (high_word <<16) + low_word
    if print_en==1: print("\n...FW Hash Code : 0x%06X\n" %(hash_code)),
    return hash_code
####################################################################################################
# This function reads CRC CODE for the FW already loaded in Serdes
# 
####################################################################################################
def fw_crc(print_en=1):
    wreg(Pam4Reg.fw_crc_word_addr, 0x0000) # clear readback value first before gets updated by next cmd
    wreg(Pam4Reg.fw_crc_read_en_addr, Pam4Reg.fw_crc_read_en)  # Enable reading back FW CRC Code
    time.sleep(.01)
    checksum_code  = rreg(Pam4Reg.fw_crc_word_addr)
    if print_en==1: print("\n...FW  CRC Code    : 0x%04X\n" %(checksum_code)) ,
    return checksum_code
####################################################################################################
# This function Calculates CRC CODE for the Code+ReadOnly Section of the FW in Memory
# 
####################################################################################################
def fw_crc_calc(print_en=1):
    wreg(0x9806, 0xA0F0)  # Enable reading back FW CRC Code
    time.sleep(.1)
    
    for i in range(1000):
        checksum_cmd_status = rreg(0x9806) # wait for 0x0A01 indicating command done
        if checksum_cmd_status==0x0A01: # ==0x0A01
            break

    if checksum_cmd_status != 0x0A01:
        checksum_code=-1
        if print_en==1: print("\n...CRC Calc for Code+ReadOnly Execution Failed. Expected: 0x0A01, Actual: 0x%04X\n" %(checksum_cmd_status)) ,
    else:
        checksum_code  = rreg(0x9807)
        if print_en==1: print("\n...CRC Calc for Code+ReadOnly Memory: 0x%04X\n" %(checksum_code)),
    return checksum_code
####################################################################################################
# This function reads DATE CODE for the FW already loaded in Serdes
# 
####################################################################################################
def fw_date(print_en=1):
    wreg(Pam4Reg.fw_date_word_addr, 0x0000) # clear readback value first before gets updated by next cmd
    wreg(Pam4Reg.fw_date_read_en_addr, Pam4Reg.fw_date_read_en)  # Enable Reading back FW Date Code
    time.sleep(.01)
    datecode = rreg(Pam4Reg.fw_date_word_addr)
    d=datetime.date(1970, 1, 1) + datetime.timedelta(datecode)

    if print_en==1: print "\n...FW Date Code : %04d-%02d-%02d\n" % (d.year, d.month, d.day),
    return datecode
####################################################################################################
# This function reads WATCHDOG Counter value
#
# The watchdog timer is incremented only by the FW, if FW is loaded in Serdes
# 
####################################################################################################
def fw_watchdog(count=None):
    if count != None:
        wreg(Pam4Reg.fw_watchdog_timer_addr,0x0000) # clear the counter if asked
        
    watchdog_count = rreg(Pam4Reg.fw_watchdog_timer_addr)
    return watchdog_count
    
fw_tick=fw_watchdog

####################################################################################################
# This function checks if a FW is loaded
#
# Returns 0: if FW is not loaded
# Returns 1: if FW is loaded
#
####################################################################################################
def fw_loaded(print_en=1):    
    val0 = fw_hash(print_en=0)           # check the hash code to see if it valid counter to see if FW is incrementing it
    if val0==0 or val0==0xFFFFFF:                # A quick check for zero value
        fw_loaded_stat=0       # hash code counter is zero, FW is not loaded
        if print_en==1: print("\n...Slice %d has no FW Loaded!"%gSlice)
    else:
        fw_loaded_stat=1       # hash code is not zero, a FW not loaded
        if print_en==1: 
            print("\n...Slice %d has FW Loaded" %(gSlice)),
            print fw_info(print_en=0)
        
    return fw_loaded_stat 
####################################################################################################
# This function checks if a FW is loaded and is running?
#
# Returns 0: if FW is not running or not loaded
# Returns 1: if FW is running
#
####################################################################################################
def fw_running():    
    val0 = fw_watchdog()       # check the watchdog counter to see if FW is incrementing it
    if val0==0:                # A quick check for zero value
        fw_running_stat=0      # watchdog counter is zero, FW is not loaded       
    else:
        time.sleep(1.8)        # wait more than one second
        val1 = fw_watchdog()   # check the watchdog counter once more
        if val1 > val0:        # watchdog counter is moving, then FW is loaded and running
            #wreg(fw_load_magic_word_addr, fw_load_magic_word) # if FW is loaded but magic word is corrupted, correct it
            fw_running_stat=1  # watchdog counter is moving, then FW is loaded and running
        else:
            fw_running_stat=0   # watchdog counter is not moving, then FW is not loaded or halted
            fw_watchdog(0)      # clear the counter so the next fw_running check goes faster            

    return fw_running_stat 
####################################################################################################
# This function removes any FW already loaded in Serdes
# 
####################################################################################################
def fw_unload(slice=None, print_en=1):

    slices = get_slice_list(slice)

    for slc in slices:
        sel_slice(slc)      
        wreg(Pam4Reg.fw_unload_addr, Pam4Reg.fw_unload_word) # Enable Unloading the FW from Slice
        cpu_reset() # Need to reset CPU 
        time.sleep(0.1)
        wreg(Pam4Reg.fw_unload_addr, 0) # Clear the Unload register after cpu reset

        for ln in range(16):
            for core in [Pam4Reg, NrzReg]:
                wreg(core.rx_bp1_st_addr, 0, ln)
                wreg(core.rx_bp1_en_addr, 0, ln)
                wreg(core.rx_sm_cont_addr, 0, ln)
                time.sleep(0.001)
                wreg(core.rx_sm_cont_addr, 1, ln)

        if print_en: print("\n...Slice %d FW Unloaded"%slc),

####################################################################################################
def fw_cmd(cmd):
    wreg(c.fw_cmd_addr, cmd) # fw_cmd_addr = 0x9806
    for i in range(1000):
        result=rreg(c.fw_cmd_addr) # fw_cmd_addr = 0x9806
        if result!=cmd: # ==0x800 or result==0xb00:
            break
    if result == 0x302:
        result=-1
        #print("*** fw_cmd %04x Failed. Return Status = 0x%04X" %(cmd, result))
    #print("*** FW Command %04x failed. is firmware loaded?" % cmd)
    return result

####################################################################################################
# Get FW Debug Information
####################################################################################################
def fw_debug_info(section=2, index=2,lane=None):
    lanes = get_lane_list(lane)
    result = {}
    timeout=0.2 
    for ln in lanes:
        result[ln] = fw_debug_cmd(section, index, ln)
        
    return result
####################################################################################################
# Get FW Debug Information
####################################################################################################
def fw_debug(section=None, index=None,lane=None):

    if section==None:         sections=[2]
    if type(section)==int:    sections=[section]
    elif type(section)==list: sections=section
    
    if index==None:           indices=[2]
    if type(index)==int:      indices=[index]
    elif type(index)==list:   indices=index
    
    lanes = get_lane_list(lane)
    result = {}
    timeout=0.2 
    for sec in sections:
        for idx in indices:
            for ln in lanes:
                result[ln].append(fw_debug_cmd(sec, idx, ln))
        
    return result
####################################################################################################
# Get FW Debug Information
####################################################################################################
def fw_debug_cmd(section=2, index=7, lane=0):
    timeout=0.2
    result=0    
    cmd = 0xB000 + ((section&0xf)<<4) + lane
    wreg(c.fw_cmd_detail_addr, index) # fw_cmd_detail_addr = 0x9807
    status = fw_cmd(cmd)   # fw_cmd_addr = 0x9806
    if(status!=0x0b00+section):
        #print("FW Debug CMD Section %d, Index %d, for Lane %s failed with code 0x%04x" %(section, index, lane_name_list[lane],status))
        result = -1
    else:
        result = rreg(c.fw_cmd_detail_addr) # fw_cmd_detail_addr = 0x9807
        
    return result
def fw_debug_tbd(val = None, lane = None, print_en=1):
    lanes = get_lane_list(lane)
    result = {}
    for ln in lanes:
        get_lane_mode(ln)
        c = Pam4Reg if lane_mode_list[ln].lower() == 'pam4' else NrzReg  
        if val != None:
            wreg(c.rx_kp_ow_addr, val, ln)
            wreg(c.rx_kp_owen_addr, 1, ln)
        
        ted_v = rreg(c.rx_ted_en_addr, ln)
        kp_v  = rreg(c.rx_kp_ow_addr, ln)
        result[ln] = ted_v, kp_v
        
    if print_en: # Print Status
        print("\nSlice %d, Lane:"%(sel_slice())),
        for ln in range(len(lane_name_list)):
            print(" %2s" %(lane_name_list[ln])),
        get_lane_mode('all')
        print("\n      CDR TED:"),
        for ln in range(len(lane_name_list)):
            c = Pam4Reg if lane_mode_list[ln] == 'pam4' else NrzReg
            ted_v = rreg(c.rx_ted_en_addr, ln)
            if lane_mode_list[ln] == 'pam4':
                print(" %2d"  %(ted_v)),
            else:
                print("  -"),
            
        print("\n      CDR Kp :"),
        for ln in range(len(lane_name_list)):
            c = Pam4Reg if lane_mode_list[ln] == 'pam4' else NrzReg
            kp_v  = rreg(c.rx_kp_ow_addr, ln)
            print(" %2d" %(kp_v)),
    else:
        if result != {}: return result        
####################################################################################################
# Get FW config Information
####################################################################################################
def fw_config_cmd(config_cmd=0x8090,config_detail=0x0000):  
    wreg(c.fw_cmd_detail_addr, config_detail) # fw_cmd_detail_addr = 0x9807
    status = fw_cmd(config_cmd)   # fw_cmd_addr = 0x9806
    if(status==0x302): # Retry sending config command one more time
        #print("FW Config CMD 0x%04X, Config Detail: 0x%04X, Failed with Status 0x%04X. RETRYING..."%(config_cmd, config_detail ,status))
        status = fw_cmd(config_cmd)   # fw_cmd_addr = 0x9806
    
    if(status==0x302): # Failed again on the second retry 
        status=-1
        #print("FW Config CMD 0x%04X, Config Detail: 0x%04X, Failed with Status 0x%04X. AFTER RETRY"%(config_cmd, config_detail ,status))

        
    return status
    
####################################################################################################
#
# Here is how to get a lane speed
# 1. For address 0x9807, write 0x0004
# 2. For address 0x9806, write 0xB00x (x is the lane number, 0 to 0xF)
# 3. Read 0x9807 for return value.
#
# The return value would be 0x00~0x06.
# 0x00 -> ??? 
# 0x01 -> 10G 
# 0x02 -> 20G
# 0x03 -> 25G
# 0x04 -> 26G
# 0x05 -> 28G
# 0x06 -> ???
# 0x07 -> ???
# 0x08 -> 50G
# 0x09 -> 53G
# 0x0A -> 56G
#
####################################################################################################
def fw_lane_speed(lane=None):
              #[ 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,  0x08,  0x09,  0x0A, 0x0B ]
    speed_list=['OFF','10G','20G','25G','26G','28G','53G','07?', '51G', '53G', '56G','0x0B?']
    mode_list =['off','nrz','nrz','nrz','nrz','nrz','pam4','07?','pam4','pam4','pam4','????']
    lanes = get_lane_list(lane)
    result = {}    
    for ln in lanes:
        speed_index= fw_debug_cmd(section=0,index=4,lane=ln)   
        result[ln] = [mode_list[speed_index],speed_list[speed_index]]
        #result[ln] = speed_index
    return result
####################################################################################################
# show current mode assigned to the lane by the FW
#
# index of the mode:
# 0: set_mode before opt
# 1: opt_mode
#
# mode:
# 0: lane in OFF
# 1: lane is in NRZ mode
# 2: lane is in PAM4 mode
####################################################################################################
def fw_lane_mode (lane=None):

    lanes = get_lane_list(lane)
    result = {}
    # print current fw_modes of the lanes
    dbg_mode=0
    dbg_cmd=c.fw_debug_info_cmd # 0xb000
    mode_def=['OFF','NRZ','PAM4']
    mode_idx=[0,1]
    opt_status_def=['-','OPT']
    opt_status_bit=[0,1]
  
    for ln in lanes:
        for idx in range(2): # get both fw modes [0]:set_mode [1]: opt_mode
            mode_idx[idx] = fw_debug_cmd(section=0, index=idx, lane=ln) # for index=1 ===> 0: OFF, 1: NRZ, 2: PAM4
        opt_status_bit = (rreg(c.fw_opt_done_addr) >> ln) & 0x0001
        result[ln] = (mode_def[mode_idx[0]], mode_def[mode_idx[1]], opt_status_def[opt_status_bit])
    return result
####################################################################################################
# pause/restart FW in target lane(s)
#
#
####################################################################################################
def fw_pause (fw_mode=None,lane=None, print_en=1):

    if not fw_loaded(print_en=0):
        print("\n*** No FW Loaded. Skipping fw_pause() \n")
        return

    top_fw_addr      = 9
    serdes_fw_addr   = 8
    tracking_fw_addr = 128
    ffe_fw_addr      = 115
    
    off_def = ['OFF','DIS','PAUSE','STOP']
    mode_def= ['off','ON']

    lanes = get_lane_list(lane)
    result = {}

    if fw_mode!=None: # Write FW Registers to pause/restart FW
        en = 0 if any(i in fw_mode.upper() for i in off_def ) else 1
        if en==0: 
            print "\n...PAUSED FW on Lanes: " + str(lanes)
        else: 
            print "\n...Restarted FW on Lanes: " + str(lanes)

        for ln in lanes:
            val1=fw_reg(addr=top_fw_addr, print_en=0)[top_fw_addr   ]
            val2=fw_reg(addr=serdes_fw_addr,print_en=0)[serdes_fw_addr]
            val3=fw_reg(addr=tracking_fw_addr, print_en=0)[tracking_fw_addr   ]
            val4=fw_reg(addr=ffe_fw_addr, print_en=0)[ffe_fw_addr   ]

            ####### Proper sequence to Pause FW    
            if en==0: 
                ### Pause Top FW     
                if any(i in fw_mode.upper() for i in ['TOP','ALL']):   
                    val = (val1 & ~(1<<ln)) | (en<<ln)
                    fw_reg(addr=top_fw_addr, data=val, print_en=0)
                ### Pause FFE FW ONLY (Keep Tracking FW 128 on to get ISI or PAM4 DFE taps)           
                if any(i in fw_mode.upper() for i in ['FFE']):   
                    val = (val4 & ~(1<<ln)) | (~en<<ln) # set lane bit to '1' to disable FFE FW
                    fw_reg(addr=ffe_fw_addr, data=val, print_en=0)
                ### Pause TRACKING FW
                ### if Pausing Serdes FW, Pause TRACKING FW first                 
                if any(i in fw_mode.upper() for i in ['SERDES','PHY','TRK','TRACKING','ALL']):   
                    val = (val3 & ~(1<<ln)) | (en<<ln)
                    fw_reg(addr=tracking_fw_addr, data=val, print_en=0)
                ### Pause Serdes FW     
                if any(i in fw_mode.upper() for i in ['SERDES','PHY','ALL']):             
                    val = (val2 & ~(1<<ln)) | (en<<ln)
                    fw_reg(addr=serdes_fw_addr, data=val, print_en=0)
                    ### Other preprations for "No-Serdes FW" mode
                    bp1(0,0,lane=ln) # Clear Breakpoint 1
                    bp2(0,0,lane=ln) # Clear Breakpoint 2
                    sm_cont(lane=ln) # Continue state machine
                    
            ####### Proper sequence to Restart FW    
            else: 
                ### Restart Serdes FW     
                ### if Restarting Serdes FW, Start TRACKING FW "after" Serdes FW    
                if any(i in fw_mode.upper() for i in ['SERDES','PHY','ALL']):             
                    val = (val2 & ~(1<<ln)) | (en<<ln)
                    fw_reg(addr=serdes_fw_addr, data=val, print_en=0)
                ### Restart TRACKING FW                      
                if any(i in fw_mode.upper() for i in ['SERDES','PHY','TRK','TRACKING','ALL']):   
                    val = (val3 & ~(1<<ln)) | (en<<ln)
                    fw_reg(addr=tracking_fw_addr, data=val, print_en=0)
                ### Restart Top FW     
                if any(i in fw_mode.upper() for i in ['TOP','ALL']):   
                    val = (val1 & ~(1<<ln)) | (en<<ln)
                    fw_reg(addr=top_fw_addr, data=val, print_en=0)
                ### Restart FFE FW last if it was off ( Tracking FW 128 asked to be back on)           
                if any(i in fw_mode.upper() for i in ['FFE','ALL','SERDES','TRK','TRACKING']):   
                    val = (val4 & ~(1<<ln)) | (~en<<ln) # set lane bit to '0' to enable FFE FW
                    fw_reg(addr=ffe_fw_addr, data=val, print_en=0)
    
    ### Read the updated values for the FW Registers        
    for ln in range(16): 
        status1=(fw_reg(addr=top_fw_addr,   print_en=0)[top_fw_addr   ]>>ln) & 1
        status2=(fw_reg(addr=serdes_fw_addr,print_en=0)[serdes_fw_addr]>>ln) & 1
        status3=(fw_reg(addr=tracking_fw_addr,   print_en=0)[tracking_fw_addr   ]>>ln) & 1
        status4=(fw_reg(addr=ffe_fw_addr,   print_en=0)[ffe_fw_addr   ]>>ln) & 1
        result[ln]=status1,status2,status3,status4
        
    ### Print FW Pause/Active Status for all lanes     
    if print_en: 
        print("\n------------------------"),
        print("  FW Pause/Off or Active/ON Status Per Lane"),
        print("  ------------------------"),
        print("\n...       Lane:"),
        for ln in range(16):
            print(" %3s" %(lane_name_list[ln])),

        print("\n...     Top FW:"),
        for ln in range(16):
            print(" %3s" %(mode_def[result[ln][0]])),
        print("\n...  Serdes FW:"),
        for ln in range(16):
            print(" %3s" %(mode_def[result[ln][1]])),
        print("\n...Tracking FW:"),
        for ln in range(16):
            print(" %3s" %(mode_def[result[ln][2]])),
        print("\n...     FFE FW:"),
        for ln in range(16):
            print(" %3s" %(mode_def[~result[ln][3]])),
        fw_reg(addr=[top_fw_addr,serdes_fw_addr,tracking_fw_addr,ffe_fw_addr])
    else:
        return result
####################################################################################################
# Use FW Command to Squelch or Unsquelch TX output
#
#
####################################################################################################
def fw_tx_squelch(mode=None,lane=None, print_en=1):

    if not fw_loaded(print_en=0):
        print("\n*** No FW Loaded. Skipping fw_tx_squelch().\n")
        return

    squelch_en_cmd      = 0x7011
    squelch_dis_cmd     = 0x7010
    squelch_status_addr = 0x98cd
    hw_tx_control_addr  = 0xA0
    
    squelch_en_def  = ['ON','EN','SQ','SQUELCH']
    squelch_dis_def = ['OFF','DIS','UNSQ','UNSQUELCH']
    mode_def= ['dis','EN']

    lanes = get_lane_list(lane)
    result = {}
    
    if print_en: print("\n-----------------------------"),
    if print_en: print("  FW TX Squelch/En or Unsquelch/Dis Status Per Lane"),
    if print_en: print("  -----------------------------"),
    if print_en: print("\n           Lane:"),
    if print_en: 
        for ln in range(16):
            print(" %4s" %(lane_name_list[ln])),

    if mode!=None: # Write FW Commands
        for ln in lanes:
            if any(i in mode.upper() for i in squelch_en_def):   
                val = 1<<ln
                fw_config_cmd(squelch_en_cmd, val)
            if any(i in mode.upper() for i in squelch_dis_def):   
                val = 1<<ln
                fw_config_cmd(squelch_dis_cmd, val)

    for ln in range(16): # Read FW and HW Registers
        status1=(rreg(squelch_status_addr)>>ln) & 1
        status2=rreg(addr=hw_tx_control_addr,lane=ln)
        result[ln]=status1,status2
        
    if print_en: # Print Status
        print("\n FW TX Squelch :"),
        for ln in range(16):
            print(" %4s"  %(mode_def[result[ln][0]])),
        print("\n HW TX Reg 0xA0:"),
        for ln in range(16):
            print(" %04X" %(result[ln][1])),
    else:
        return result
##############################################################################
# Use FW Command to Configure a lane in Optic Mode
#
#
##############################################################################
def fw_optic_mode(mode=None,lane=None, print_en=1):

    if not fw_loaded(print_en=0):
        print("\n*** No FW Loaded. Skipping fw_tx_squelch().\n")
        return

    optic_fw_reg_addr = 20
    optic_mode_reg_val = fw_reg_rd(optic_fw_reg_addr)
    
    optic_en_def  = ['ON','EN','OPTIC']
    optic_dis_def = ['OFF','DIS','COPPER']
    mode_def= ['dis','EN']

    lanes = get_lane_list(lane)
    result = {}
    
    if print_en: print("\n-----------------------------------"),
    if print_en: print("  FW Optic Mode Status Per Lane"),
    if print_en: print("  -----------------------------------"),
    if print_en: print("\n           Lane:"),
    if print_en: 
        for ln in range(16):
            print(" %4s" %(lane_name_list[ln])),
            
        
    if mode!=None: # Write FW Commands
        for ln in lanes:
            if any(i in mode.upper() for i in optic_en_def):
                optic_mode_reg_val = (optic_mode_reg_val | (1<<ln))
            if any(i in mode.upper() for i in optic_dis_def):   
                optic_mode_reg_val = (optic_mode_reg_val & ~(1<<ln))
        fw_reg_wr (optic_fw_reg_addr, optic_mode_reg_val)
        
    for ln in range(16): # Read relavant Registers
        lane_optic_mode =(fw_reg_rd(optic_fw_reg_addr) >> ln) & 1
        result[ln]=lane_optic_mode
        
    if print_en: # Print Status
        print("\n FW Optic Mode :"),
        for ln in range(16):
            print(" %4s"  %(mode_def[result[ln]])),
    else:
        return result
##############################################################################
# wait until all lanes are done with FW-based RX ADAPTATION 
#
# Status of all lanes RX and FECs set up as gearbox:
# 0: lane's RX adaptation not done or incomplete
# 1: lane's RX adaptation complete
##############################################################################
def fw_gearbox_wait (max_wait=2000, print_en=0):

    fec_b_bypass = True if rreg([0x9857,[7,4]])==0 else False
    start_time=time.time()
    
    cnt=1
    gearbox_done=0
    final_state_count=0
    final_gearbox_state = 6 if fec_b_bypass==True else 12  # When FECB Bypassed, final Gearbox FW State = 6, else 12
    if print_en: print("\n...Waiting for FEC Locks (fec_wait=%d)      "%(fw_reg_rd(14))),
    while ( not gearbox_done ):
        if print_en: print("\b\b\b\b\b\b%4.1fs"%(time.time()-start_time)),
        if print_en: print("\n%4.1fs"%(time.time()-start_time)),
        cnt+=1
        time.sleep(.1)
        gb_state0  = fw_debug_cmd(section=3,index=0, lane=0)
        gb_state2  = fw_debug_cmd(section=3,index=0, lane=2)
        gb_state8  = fw_debug_cmd(section=3,index=0, lane=8)
        gb_state12 = fw_debug_cmd(section=3,index=0, lane=12)
        
        if print_en: 
            tx_output_a0 = hex(rreg(0xa0, 0))
            tx_output_a2 = hex(rreg(0xa0, 2))
            tx_output_b0 = hex(rreg(0xa0, 8))
            tx_output_b4 = hex(rreg(0xa0,12))
            fec_stat, fec_counts = fec_status(print_en=0)
            adapt_stat = hex(rreg(0x98c9))
            reg1=hex(rreg(0x4880));reg2=hex(rreg(0x4a80));reg3=hex(rreg(0x5880));reg4=hex(rreg(0x5a80))
            #print gb_state0,gb_state2,gb_state8,gb_state12, tx_output_a0, tx_output_a2, tx_output_b0, tx_output_b4, adapt_stat, reg1,reg2,reg3,reg4, fec_stat,
            if print_en: print gb_state0,gb_state2,gb_state8,gb_state12, adapt_stat, reg1,reg2,reg3,reg4, fec_stat,
            
        if gb_state0==final_gearbox_state and gb_state2==final_gearbox_state and gb_state8==final_gearbox_state and gb_state12==final_gearbox_state:
            if final_state_count<3:
                final_state_count+=1
            else:
                gearbox_done=1
        if cnt>max_wait :
            gearbox_done=1
    #fec_status()
    if print_en: print("...Done!\n")

    return gearbox_done
####################################################################################################
# wait until all lanes are done with FW-based RX ADAPTATION 
#
# Status of all lanes RX and FECs set up as gearbox:
# 0: lane's RX adaptation not done or incomplete
# 1: lane's RX adaptation complete
####################################################################################################
def fw_gearbox_wait_orig (max_wait=None, lane=None, print_en=1):

    lanes = get_lane_list(lane)
    start=time.time()
    adapt_done_total_prev = 0
    adapt_done_total = 0
    adapt_done_lane_prev = [0]*16
    adapt_done_lane_curr = [0]*16
    
    if max_wait==None:
        maxWait = 30
    else:
        maxWait = max_wait

    if(print_en==1):
        print("\n...Slice %d Lanes %d-%d RX Adaptation In Progress. . . \n"%(gSlice,lanes[0],lanes[-1]))
        for ln in lanes:
            print("%2s" %(lane_name_list[ln]) ),
    
    if(print_en==2): print("\n...Slice %d Lanes %d-%d RX Adaptation In Progress. . ."%(gSlice,lanes[0],lanes[-1])),
    
    while adapt_done_total < len(lanes):
        if(print_en==1): print("")
        if(print_en==2): print("."),
        adapt_done_total=0
        for ln in lanes:
            adapt_done_lane_curr[ln] = (rreg(c.fw_opt_done_addr) >> ln) & 1
            adapt_done_total += adapt_done_lane_curr[ln]
            if(print_en==2 and adapt_done_total_prev==0 and adapt_done_total>0):
                print("\n"),
            if(print_en==2 and adapt_done_lane_curr[ln]==1 and adapt_done_lane_prev[ln]==0 ):
                print("%s"%(lane_name_list[ln])),
            if(print_en==1): 
                print("%2d" %(adapt_done_lane_curr[ln]) ),
            adapt_done_lane_prev[ln] = adapt_done_lane_curr[ln]
            adapt_done_total_prev += adapt_done_total
            
        t2 = time.time() - start
        if(print_en==1): print(" %2.2fs"%t2),
        if (t2 > maxWait ):
            break

        time.sleep(.1)
                
    if(print_en==1):
        print""
        for ln in lanes:
            print("%2s" %(lane_name_list[ln]) ),
        print"\n"

    adapt_done_all = 1 if adapt_done_total==len(lanes) else 0
    
    return [adapt_done_all, t2]
    
    
           
####################################################################################################
# wait until all lnaes are done with FW-based RX ADAPTATION 
#
# Status of each lane:
# 0: lane's RX adaptation not done or incomplete
# 1: lane's RX adaptation complete
####################################################################################################
def fw_config_wait (max_wait=None, lane=None, print_en=1):

    lanes = get_lane_list(lane)
    config_done_this_lane = [0]*16
    config_done_all_lanes=0
    
    if max_wait==None:
        maxWait = 3.0
        # if any of the lanes is PAM4 mode, set the max wait to 20 seconds, otherwise 3 seconds max 
        for ln in lanes:
            #lane_speed_index = fw_debug_cmd(section=0, index=4,  lane=ln)
            curr_fw_lane_mode = fw_lane_mode (ln)
            if any('PAM4' in s for s in curr_fw_lane_mode[ln]): maxWait =8
    else:
        maxWait = max_wait

    start=time.time()
    while config_done_all_lanes < len(lanes):
        config_done_all_lanes=0
        for ln in lanes:
            config_code_this_lane= fw_debug_cmd(section=0, index=38, lane=ln)
            ### Config is GB, BM or Retimer but not PHY mode
            if config_code_this_lane<0xA0:
                config_done_this_lane[ln] = (fw_debug_cmd(section=0, index=40, lane=ln) >> ln) & 1
            #### Config is PHY Mode. Get its status register 0x98c9
            else: 
                config_done_this_lane[ln] = (rreg(c.fw_opt_done_addr) >> ln) & 1   

            config_done_all_lanes += config_done_this_lane[ln]
        t2 = time.time() - start
        if (t2 > maxWait ):
            break
        time.sleep(.1)

    config_done_all = 1 if config_done_all_lanes==len(lanes) else 0
    
    return [config_done_all, t2]

####################################################################################################
# wait until all lnaes are done with FW-based RX ADAPTATION 
#
# Status of each lane:
# 0: lane's RX adaptation not done or incomplete
# 1: lane's RX adaptation complete
####################################################################################################
def fw_adapt_wait (max_wait=None, lane=None, print_en=1):

    lanes = get_lane_list(lane)
    start=time.time()
    adapt_done_total_prev = 0
    adapt_done_total = 0
    adapt_done_reg=0
    adapt_done_reg_prev=0
    adapt_done_lane_prev = [0]*16
    adapt_done_lane_curr = [0]*16
    
    if max_wait==None:
        maxWait = 3.0
        # if any of the lanes is PAM4 mode, set the max wait to 20 seconds, otherwise 3 seconds max 
        for ln in lanes:
            curr_fw_lane_mode = fw_lane_mode (ln)
            if any('PAM4' in s for s in curr_fw_lane_mode[ln]): maxWait =8
    else:
        maxWait = max_wait

    if(print_en==1):
        print("\n...Slice %d Lanes %d-%d RX Adaptation In Progress... \n"%(gSlice,lanes[0],lanes[-1]))
        print("  "),
        for ln in lanes:
            print("%2s" %(lane_name_list[ln]) ),
        print("")
    if(print_en==2): print("\n...Slice %d Lanes %d-%d RX Adaptation In Progress..."%(gSlice,lanes[0],lanes[-1])),
    
    while adapt_done_total < len(lanes):
        adapt_done_reg_prev = adapt_done_reg
        adapt_done_reg = rreg(c.fw_opt_done_addr)
        if(print_en==1):
            print("  "),
        if(print_en==2): 
            print("\b."),            
        adapt_done_total=0
        for ln in lanes:
            adapt_done_lane_curr[ln] = (adapt_done_reg >> ln) & 1
            adapt_done_total += adapt_done_lane_curr[ln]
            if(print_en==2 and adapt_done_total_prev==0 and adapt_done_total>0):
                print("\r...Slice %d Lanes %d-%d RX Adaptation In Progress..."%(gSlice,lanes[0],lanes[-1])),
            if(print_en==2 and adapt_done_lane_curr[ln]==1 and adapt_done_lane_prev[ln]==0 ):
                print("\b%s."%(lane_name_list[ln])),
            if(print_en==1): 
                print("%2d" %(adapt_done_lane_curr[ln]) ),
            adapt_done_lane_prev[ln] = adapt_done_lane_curr[ln]
            adapt_done_total_prev += adapt_done_total

            
        t2 = time.time() - start
        if(print_en==1): 
            print(" %4.1fs"%t2),
            if(adapt_done_reg!=adapt_done_reg_prev): 
                print('\n'), # ERASE_LINE = '\x1b[2K, CURSOR_UP_ONE = '\x1b[1A'
            else:
                print('\r'),
        if (t2 > maxWait ):
            break
        time.sleep(.1)
                
    if(print_en==1):
        print("  "),
        for ln in lanes:
            print("%2s" %(lane_name_list[ln]) ),
        print("\n"),

    adapt_done_all = 1 if adapt_done_total==len(lanes) else 0
    
    return [adapt_done_all, t2]
####################################################################################################
# wait until all Bitmux Config are done with FW-based BITMUX
#
# Status of each Bitmux:
# 0: lane's Bitmux Config not done or incomplete
# 1: lane's Bitmux Config complete
####################################################################################################
def fw_bitmux_wait (lane=[0,1,2,3], max_wait=None, print_en=1):

    lanes = get_lane_list(lane)
    b_lanes=[[8,9],[10,11],[12,13],[14,15],[12,13],[14,15]]
    #print b_lanes
    start=time.time()
    bitmux_done_total = 0
    done_total_prev = 0
    done_total = 0

    bitmux_tx_reg_lane_curr = [0]*16
    bitmux_done_lane_prev = [0]*16
    bitmux_state_lane_curr = [0]*16
    bitmux_done_lane_curr = [0]*16
    adapt_done_lane_prev = [0]*16
    adapt_done_lane_curr = [0]*16
    
    if max_wait==None:
        maxWait = 10.0
    else:
        maxWait = max_wait

    if(print_en!=0):
        print("\n...Slice %d Lanes %d-%d Bitmux Config In Progress..."%(gSlice,lanes[0],lanes[-1])),

    if(print_en==1):
        print("\n  "),
        for a_ln in lanes:
            print("BM%d" %(a_ln) ),
            print("%2s" %(lane_name_list[a_ln]) ),
            for b_ln in b_lanes[a_ln]:
                print("%2s" %(lane_name_list[b_ln]) ),
            print(""),
        print("")
    
    while bitmux_done_total < len(lanes):
        if(print_en==1):
            print("  "),
        if(print_en==2): 
            print("\b."),            
            if(bitmux_done_total==0 ):
                print("\b\b\b\b..."),        
        bitmux_done_total=0
        done_total=0
        
        
        for a_ln in lanes:
            bitmux_tx_reg_lane_curr[a_ln] = rreg(0x0A0,lane=14)  # get bitmux TX registers 0xA0
            bitmux_state_lane_curr[a_ln] = fw_debug_cmd(4, 0,a_ln)   # get bitmux state number
            bitmux_done_lane_curr[a_ln] = 1 if fw_debug_cmd(8, 99,a_ln)==1 else 0   # get bitmux done for this group
            bitmux_done_total += bitmux_done_lane_curr[a_ln]
            done_total += bitmux_done_lane_curr[a_ln]
            
            adapt_done_lane_curr[a_ln] = (rreg(c.fw_opt_done_addr) >> a_ln) & 1 # get adapt done for this group
            done_total += adapt_done_lane_curr[a_ln]
            
            for b_ln in b_lanes[a_ln]:                                      # get adapt done for this group
                adapt_done_lane_curr[b_ln] = (rreg(c.fw_opt_done_addr) >> b_ln)  & 1
                done_total += adapt_done_lane_curr[b_ln]
                                          
            if(print_en==1): 
                print("%d" %(bitmux_state_lane_curr[a_ln])),
                print("%d" %(bitmux_done_lane_curr[a_ln])),
                print("%2d"  %(adapt_done_lane_curr[a_ln])),
                #print("0x%04x"  %(bitmux_tx_reg_lane_curr[a_ln])),
                for b_ln in b_lanes[a_ln]:
                    print("%2d" %(adapt_done_lane_curr[b_ln]) ),
                print(""),
            if(print_en==2):
                if(bitmux_done_lane_curr[a_ln]==1 and bitmux_done_lane_prev[a_ln]==0 ):
                    print("\b\bBM%d.."%(a_ln)),
                bitmux_done_lane_prev[a_ln]=bitmux_done_lane_curr[a_ln]
                
        t2 = time.time() - start
        if(print_en==1): 
            print(" %4.1fs"%t2),
            if(done_total!=done_total_prev): 
                print('\n'), # ERASE_LINE = '\x1b[2K, CURSOR_UP_ONE = '\x1b[1A'
            else:                
                print('\r'),
        done_total_prev = done_total
        if (t2 > maxWait ):
            break
        time.sleep(.01)
                
    if(print_en==1):
        print("  "),
        for a_ln in lanes:
            print("BM%d" %(a_ln) ),
            print("%2s" %(lane_name_list[a_ln]) ),
            for b_ln in b_lanes[a_ln]:
                print("%2s" %(lane_name_list[b_ln]) ),
            print(""),

    bitmux_done_all = 1 if bitmux_done_total==len(lanes) else 0
    
    return [bitmux_done_all, t2]
####################################################################################################
#
# "Adaptation" counter for SerDes PHY. Rolls over at 0xFFFF. 
#
####################################################################################################
def fw_adapt_cnt(lane=None):
    lanes = get_lane_list(lane)
    result = {} 
    for ln in lanes:
        if not fw_loaded(print_en=0):
            result[ln] =-1
            continue
        get_lane_mode(ln)
        if gEncodingMode[gSlice][ln][0]=='pam4':  # Lane is in PAM4 mode
            result[ln] = fw_debug_cmd(section=2,index=7,lane=ln)
        else:                                     # Lane is in NRZ mode
            result[ln] = fw_debug_cmd(section=1,index=10,lane=ln)

    return result
####################################################################################################
#
# "Re-adaptation" counter for SerDes PHY. Clears on read, saturates to 0xFFFF. 
# This increments with fw_adapt_cnt (FW does a lane-restart)
#
####################################################################################################
def fw_readapt_cnt(lane=None):
    lanes = get_lane_list(lane)
    result = {}    
    for ln in lanes:
        if not fw_loaded(print_en=0):
            result[ln] =-1
            continue
        get_lane_mode(ln)
        result[ln] = fw_debug_cmd(section=8,index=1,lane=ln)

    return result
####################################################################################################
#
# "Link lost" counter for SerDes PHY. Clears on read, saturates to 0xFFFF.
#
####################################################################################################
def fw_link_lost_cnt(lane=None):
    lanes = get_lane_list(lane)
    result = {}        
    for ln in lanes:
        if not fw_loaded(print_en=0):
            result[ln] =-1
            continue
        get_lane_mode(ln)
        result[ln] = fw_debug_cmd(section=8,index=0,lane=ln)

    return result
####################################################################################################
#
# "Gearbox FEC link lost" count, Clears on read, saturates to 0xFFFF.
#
####################################################################################################
def fw_gearbox_lost_cnt(lane=None):
    lanes = get_lane_list(lane)
    result = {}    
    for ln in lanes:
        if not fw_loaded(print_en=0):
            result[ln] =-1
            continue
        get_lane_mode(ln)
        result[ln] = fw_debug_cmd(section=11,index=0,lane=ln) 
    return result
####################################################################################################
#
# "Adaptation" counter for SerDes PHY. Rolls over at 0xFFFF. 
#
####################################################################################################
def fw_chan_est(lane=None):
    lanes = get_lane_list(lane)
    result = {}    
    for ln in lanes:
        if not fw_loaded(print_en=0):
            result[ln] =gChanEst[gSlice][ln]
            continue
        get_lane_mode(ln)
        [lane_mode,lane_speed]=fw_lane_speed(ln)[ln]
        sect = 2 if lane_mode.upper()=='PAM4' else 1
        chan_est =(fw_debug_info(section=sect, index=2,lane=ln)[ln]) / 256.0
        of       = fw_debug_info(section=sect, index=4,lane=ln)[ln]
        hf       = fw_debug_info(section=sect, index=5,lane=ln)[ln]            
        gChanEst[gSlice][ln]=[chan_est,of,hf]
        result[ln] = [chan_est,of,hf]
    return result
####################################################################################################
def fw_reg_rd(addr):
    c = Pam4Reg
    wreg(c.fw_cmd_detail_addr, addr) # fw_cmd_detail_addr = 0x9807
    result=fw_cmd(0xe010)            # fw_cmd_addr = 0x9806
    if result!=0x0e00:
        print("*** FW Register read error, code=%04x" % result)
    return rreg(c.fw_cmd_status_addr) # fw_cmd_status_addr = 0x98C7

####################################################################################################
def fw_reg_wr(addr, data):
    c = Pam4Reg
    wreg(c.fw_cmd_detail_addr, addr) # fw_cmd_detail_addr = 0x9807
    wreg(c.fw_cmd_status_addr, data) # fw_cmd_status_addr = 0x98C7
    result=fw_cmd(0xe020)            # fw_cmd_addr = 0x9806
    if result!=0x0e00:
        print("*** FW Register write error, code=%04x" % result)
        
####################################################################################################
def fw_reg(addr=None, data=None, print_en=1):
    
    
    if addr==None:
        addr_list=range(201)    # read all FW registers
        data=None               # Make sure not to write more than one address at a time. Just read FW registers and exit!
    elif type(addr)==int:       
        addr_list=[addr]        # read single FW register
    elif type(addr)==list:    
        addr_list=addr          # read list of FW registers
        data=None               # Make sure not to write more than one address at a time. Just read FW registers and exit!
 
    result = {}
    c = Pam4Reg

    str=""
    #### FW Reg Write if data-to-write is not given
    if data!=None:
        wreg(c.fw_cmd_detail_addr, addr_list[0]) # fw_cmd_detail_addr = 0x9807
        wreg(c.fw_cmd_status_addr, data) # fw_cmd_status_addr = 0x98C7
        cmd_status=fw_cmd(0xe020)            # fw_cmd_addr = 0x9806
        if cmd_status!=0x0e00:
            print("*** FW Register write error: Addr %d Code=0x%04x" %(addr, cmd_status))
            return False
    
    line_separator= "\n#+-------------------------+"
    title         = "\n#+ FWReg |      Value      |"
    str += line_separator + title + line_separator
    #### FW Reg Read 
    for reg_addr in addr_list:
        wreg(c.fw_cmd_detail_addr, reg_addr) # fw_cmd_detail_addr = 0x9807
        cmd_status=fw_cmd(0xe010)            # fw_cmd_addr = 0x9806
        if cmd_status!=0x0e00:
            # Undefined FW register address or the read of a "defined" FW register failed
            #print("*** FW Register read error: Addr %d Code=0x%04x" % (reg_addr,cmd_status))
            break
        else:
            result[reg_addr] = rreg(c.fw_cmd_status_addr)
            str += ("\n#|  %3d  | 0x%04x  (%-5d) |"%(reg_addr,result[reg_addr],result[reg_addr]))
    
    str += line_separator
    if print_en == 1: 
        print str
    else:
        return result

####################################################################################################
# Instruct the FW to configure a lane (PHY) to a mode (and a speed)
# 
# Options for 'mode': 'nrz'      :  NRZ 25G (25.78125Gbps)
#                   : 'pam4'     : PAM4 53G (53.125  Gbps)
#
#                   : 'nrz-10G'  :  NRZ 10G (10.3125 Gbps)
#                   : 'nrz-20G'  :  NRZ 20G (20.6250 Gbps)
#                   : 'nrz-25G'  :  NRZ 25G (25.78125Gbps)
#                   : 'nrz-26G'  :  NRZ 26G (26.5625 Gbps)
#                   : 'nrz-28G'  :  NRZ 28G (28.125  Gbps)
#
#                   : 'pam4-51G' : PAM4 50G (51.5625 Gbps)
#                   : 'pam4-50G' : PAM4 53G (53.125  Gbps)
#                   : 'pam4-53G' : PAM4 53G (53.125  Gbps)
#                   : 'pam4-56G' : PAM4 56G (56.25   Gbps)
#
####################################################################################################
def fw_config_lane (mode=None, datarate=None, lane=None):

    if not fw_loaded(print_en=0):
        print("\n*** No FW Loaded. Skipping fw_config_lane().\n")
        return
    
    lanes = get_lane_list(lane)
    curr_fw_lane_mode = fw_lane_mode (lanes) # learn which mode (nrz/pam4/off) each lane is in now
    
    speed_str_list =['10','20','25','26','28','51','50','53','56'] # speed as part of mode argument
    speed_code_list=[0x11,0x22,0x33,0x44,0x55,0x88,0x99,0x99,0xAA] # speed codes to be written to 0x9807[7:0]
    
    if mode==None: # no arguments? then just print current fw_modes of the lanes and exit
        print""
        for ln in lanes:
            print(" %4s" %(lane_name_list[ln])),
        for idx in [1,2]: # show both fw modes [0]:set_mode [1]: opt_mode [2]: adapt_done flag
            print""
            for ln in lanes:
                print(" %4s"%curr_fw_lane_mode[ln][idx]),
    else:  # configure (either activate or turn off) the lanes          
        if 'NRZ' in mode.upper():
            mode_code_cmd = 0x80C0               # command to activate lane in NRZ
            speed_code_cmd = 0x33                # default NRZ speed is 25G                      
            if datarate!=None:  mode=mode+str(int(round((datarate-1.0)/5.0)*5.0)) # add datarate to mode so we can find it in the speed codes
            for i in range(len(speed_str_list)): # if NRZ speed is specified, take it
                if speed_str_list[i] in mode: speed_code_cmd = speed_code_list[i]
  
        elif 'PAM4' in mode.upper():
            mode_code_cmd = 0x80D0               # command to activate lane in PAM4
            speed_code_cmd = 0x99                # default PAM4 speed is 53G
            for i in range(len(speed_str_list)): # if PAM4 speed is specified, take it
                if speed_str_list[i] in mode: speed_code_cmd = speed_code_list[i]
                
        elif 'OFF' in mode.upper():              # command to deactivate lane (Turn it OFF)
            mode_code_cmd = 0x90D0 if(curr_fw_lane_mode[lanes[0]][1] == 'PAM4') else 0x90C0
            speed_code_cmd = 0x00                # For deactivating the lane, speed code does not matter

        else:
            for ln in lanes:
                print("\n***Slice %d Lane %s FW Config: selected mode is invalid  => '%s'" %(gSlice,lane_name_list[ln],mode.upper())),
            return
                
        ################ Destroy the target lanes before programming them to target mode
        for ln in lanes:             
            off_cmd = 0x90D0 if(curr_fw_lane_mode[ln][1] == 'PAM4') else 0x90C0
            result=fw_config_cmd(config_cmd=off_cmd+ln,config_detail=0x0000) 
            #print("\n...Slice %d Lane %s FW freed up lane before reconfiguring it. (OffCode 0x%04X, Error Code 0x%04x)" %(gSlice,lane_name_list[ln],off_cmd+ln,result)),
            if (result!=c.fw_config_lane_status): # fw_config_lane_status=0x800
                print("\n***Slice %d Lane %s: FW could not free up lane before reconfiguring it. (Error Code 0x%04x)" %(gSlice,lane_name_list[ln],result)),
        
        for ln in lanes:             
            wreg([0xa0,[15,11]],0x1d,ln) # '11101' Make sure TX output is not squelched
            
        ############# Now, configure the lane per user's target mode (either activate or deactivate)
        for ln in lanes:             
            result=fw_config_cmd(config_cmd=mode_code_cmd+ln,config_detail=speed_code_cmd) 
            #print("\n...Slice %d Lane %s FW_CONFIG_LANE to Active Mode. (SpeedCode 0x9807=0x%04X, ActivateCode 0x9806=0x%04X, ExpectedStatus:0x%0X, ActualStatus=0x%04x)" %(gSlice,lane_name_list[ln],speed_code_cmd, mode_code_cmd+ln,c.fw_config_lane_status,result)),
            if (result!=c.fw_config_lane_status): # fw_config_lane_status=0x800
                print("\n***Slice %d Lane %s FW_CONFIG_LANE to Active Mode Failed. (SpeedCode 0x9807=0x%04X, ActiveCode 0x9806=0x%04X, ExpectedStatus:0x%0X, ActualStatus=0x%04x)" %(gSlice,lane_name_list[ln],speed_code_cmd, mode_code_cmd+ln,c.fw_config_lane_status,result)),

                     
####################################################################################################  
# FW to program Gearbox mode, A-side PAM4, B-side: NRZ
#
#
####################################################################################################
def fw_config_gearbox_100G(A_lanes=[0,1], fec_b_byp=0):
    
    if not fw_loaded(print_en=0):
        print("\n...FW Gearbox 100G-2 : FW not loaeded. Not executed!"),
        return
        
    print_en=1
    #fec_reset() # reset all 8 FECs and clear their Align Markers
    
    # For 100G-2 Gearbox mode, 3 options supported for A-Lane groups 
    group0_100G=[0,1] # A_lanes group 1 -> [A0,A1] <-> [ 8, 9,10,11]
    group1_100G=[2,3] # A_lanes group 2 -> [A2,A3] <-> [12,13,14,15]
    group2_100G=[4,5] # A_lanes group 3 -> [A4,A5] <-> [12,13,14,15]
    
    #Determine the corresponding B-Lanes for each group of A-Lanes
    B_lanes=[]
    if all(elem in A_lanes for elem in group0_100G):  # If A_lanes contains [0,1]
        B_lanes+=[8,9,10,11]
    if all(elem in A_lanes for elem in group1_100G):  # If A_lanes contains [2,3]
        B_lanes+=[12,13,14,15]
    elif all(elem in A_lanes for elem in group2_100G): # If A_lanes contains [4,5]
        B_lanes+=[12,13,14,15]
    #else:
    #    print("\n*** 100G-2 Gearbox Setup: Invalid Target A-Lanes specified!\n")
    #    return
    
    lanes = sorted(list(set(A_lanes + B_lanes)))
    prbs_mode_select(lane=lanes, prbs_mode='functional')

    if all(elem in A_lanes for elem in group0_100G):  # If A_lanes contains [0,1]
        fw_config_cmd(config_cmd=0x9090,config_detail=0x0000) # 0x9090 = First, FW destroy any instances of these lanes being already used      
    if all(elem in A_lanes for elem in group1_100G):  # If A_lanes contains [2,3]
        fw_config_cmd(config_cmd=0x9091,config_detail=0x0000) # 0x9091 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9092,config_detail=0x0000) # 0x9092 = First, FW destroy any instances of these lanes being already used     
    elif all(elem in A_lanes for elem in group2_100G): # If A_lanes contains [4,5]
        fw_config_cmd(config_cmd=0x9092,config_detail=0x0000) # 0x9091 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9091,config_detail=0x0000) # 0x9092 = First, FW destroy any instances of these lanes being already used

    if all(elem in A_lanes for elem in group0_100G):  # If A_lanes contains [0,1]
        if fec_b_byp==False:
            if print_en: print("\n...FW Gearbox 100G-2: Lanes A0-A1 to B0-B3 with FEC_A0/FEC_B0..."),
            fw_config_cmd(config_cmd=0x8090,config_detail=0x0000) # 0x8090 = FW Activate Gearbox 100G-2 for Lanes A0/A1        
        else:
            if print_en: print("\n...FW Gearbox 100G-2: Lanes A0-A1 to B0-B3 with FEC_A0 (FEC_B0 Bypassed)..."),
            fw_config_cmd(config_cmd=0x8098,config_detail=0x0000) # no fec 0x8098 = FW Activate Gearbox 100G-2 for Lanes A0/A1        
    if all(elem in A_lanes for elem in group1_100G):  # If A_lanes contains [2,3]
        if fec_b_byp==False:
            if print_en: print("\n...FW Gearbox 100G-2: Lanes A2-A3 to B4-B7 with FEC_A2/FEC_B2..."),
            fw_config_cmd(config_cmd=0x8091,config_detail=0x0000) # 0x8091 = FW Activate Gearbox 100G-2 for Lanes A2/A3       
        else:
            if print_en: print("\n...FW Gearbox 100G-2: Lanes A2-A3 to B4-B7 with FEC_A2 (FEC_B2 Bypassed)..."),
            fw_config_cmd(config_cmd=0x8099,config_detail=0x0000) # no fec 0x8099 = FW Activate Gearbox 100G-2 for Lanes A2/A3       
    elif all(elem in A_lanes for elem in group2_100G): # If A_lanes contains [4,5]
        if fec_b_byp==False:
            if print_en: print("\n...FW Gearbox 100G-2: Lanes A4-A5 to B4-B7 with FEC_A2/FEC_B2..."),
            fw_config_cmd(config_cmd=0x8092,config_detail=0x0000)# 0x9092 = FW Activate Gearbox 100G-2 for Lanes A4/A5
        else:
            if print_en: print("\n...FW Gearbox 100G-2: Lanes A4-A5 to B4-B7 with FEC_A2 (FEC_B2 Bypassed)..."),
            fw_config_cmd(config_cmd=0x809A,config_detail=0x0000) # no fec 0x809A = FW Activate Gearbox 100G-2 for Lanes A4/A5       
    
    if print_en: print("Done!")
    #fec_status()
##############################################################################  
# FW to program Gearbox mode, A-side PAM4, B-side: NRZ
#
#
##############################################################################
def fw_config_gearbox_100G_LT(A_lanes=[0,1], fec_b_byp=0):
    
    if not fw_loaded(print_en=0):
        print("\n...FW Gearbox 100G-2 to 25G NRZ_ANLT: FW not loaded. Not executed!"),
        return
    print_en=1
    #fec_reset() # reset all 8 FECs and clear their Align Markers
    
    # For 100G-2 Gearbox mode, 3 options supported for A-Lane groups 
    group0_100G=[0,1] # A_lanes group 1 -> [A0,A1] <-> [ 8, 9,10,11]
    group1_100G=[2,3] # A_lanes group 2 -> [A2,A3] <-> [12,13,14,15]
    group2_100G=[4,5] # A_lanes group 3 -> [A4,A5] <-> [12,13,14,15]
    
    #Determine the corresponding B-Lanes for each group of A-Lanes
    B_lanes=[]
    if all(elem in A_lanes for elem in group0_100G):  # If A_lanes contains [0,1]
        B_lanes+=[8,9,10,11]
    if all(elem in A_lanes for elem in group1_100G):  # If A_lanes contains [2,3]
        B_lanes+=[12,13,14,15]
    elif all(elem in A_lanes for elem in group2_100G): # If A_lanes contains [4,5]
        B_lanes+=[12,13,14,15]
    #else:
    #    print("\n*** 100G-2 Gearbox Setup: Invalid Target A-Lanes specified!\n")
    #    return
    
    lanes = sorted(list(set(A_lanes + B_lanes)))
    prbs_mode_select(lane=lanes, prbs_mode='functional')

    if all(elem in A_lanes for elem in group0_100G):  # If A_lanes contains [0,1]     
        fw_config_cmd(config_cmd=0x9090,config_detail=0x0000) # 0x9090 = First, FW destroy any instances of these lanes being already used      
    if all(elem in A_lanes for elem in group1_100G):  # If A_lanes contains [2,3]
        fw_config_cmd(config_cmd=0x9091,config_detail=0x0000) # 0x9091 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9092,config_detail=0x0000) # 0x9092 = First, FW destroy any instances of these lanes being already used     
    elif all(elem in A_lanes for elem in group2_100G): # If A_lanes contains [4,5]
        fw_config_cmd(config_cmd=0x9092,config_detail=0x0000) # 0x9091 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9091,config_detail=0x0000) # 0x9092 = First, FW destroy any instances of these lanes being already used

    if all(elem in A_lanes for elem in group0_100G):  # If A_lanes contains [0,1]
        if fec_b_byp==False:
            if print_en: print("\n...FW Gearbox 100G-2 to 25G NRZ_ANLT: Lanes A0-A1 to B0-B3 with FEC_A0/FEC_B0..."),
            fw_config_cmd(config_cmd=0x8090,config_detail=0x0200) # 0x8090 = FW Activate Gearbox 100G-2 for Lanes A0/A1        
        else:
            if print_en: print("\n...FW Gearbox 100G-2 to 25G NRZ_ANLT: Lanes A0-A1 to B0-B3 with FEC_A0 (FEC_B0 Bypassed)..."),
            fw_config_cmd(config_cmd=0x8098,config_detail=0x0200) # no fec 0x8098 = FW Activate Gearbox 100G-2 for Lanes A0/A1        
    if all(elem in A_lanes for elem in group1_100G):  # If A_lanes contains [2,3]
        if fec_b_byp==False:
            if print_en: print("\n...FW Gearbox 100G-2 to 25G NRZ_ANLT: Lanes A2-A3 to B4-B7 with FEC_A2/FEC_B2..."),
            fw_config_cmd(config_cmd=0x8091,config_detail=0x0200) # 0x8091 = FW Activate Gearbox 100G-2 for Lanes A2/A3       
        else:
            if print_en: print("\n...FW Gearbox 100G-2 to 25G NRZ_ANLT: Lanes A2-A3 to B4-B7 with FEC_A2 (FEC_B2 Bypassed)..."),
            fw_config_cmd(config_cmd=0x8099,config_detail=0x0200) # no fec 0x8099 = FW Activate Gearbox 100G-2 for Lanes A2/A3       
    elif all(elem in A_lanes for elem in group2_100G): # If A_lanes contains [4,5]
        if fec_b_byp==False:
            if print_en: print("\n...FW Gearbox 100G-2 to 25G NRZ_ANLT: Lanes A4-A5 to B4-B7 with FEC_A2/FEC_B2..."),
            fw_config_cmd(config_cmd=0x8092,config_detail=0x0200)# 0x9092 = FW Activate Gearbox 100G-2 for Lanes A4/A5
        else:
            if print_en: print("\n...FW Gearbox 100G-2 to 25G NRZ_ANLT: Lanes A4-A5 to B4-B7 with FEC_A2 (FEC_B2 Bypassed)..."),
            fw_config_cmd(config_cmd=0x809A,config_detail=0x0200) # no fec 0x809A = FW Activate Gearbox 100G-2 for Lanes A4/A5       
    
    if print_en: print("Done!")
    #fec_status()
def fw_config_gearbox_50G(A_lanes=[0,1,2,3], fec_b_byp=False):
    
    if not fw_loaded(print_en=0):
        print("\n...FW Gearbox 50G-1 : FW is not loaded. Not executed!"),
        return
        
    print_en=1
    
    # For 50G-2 Gearbox mode, 3 options supported for A-Lane groups 
    group0_50G=[0] # A_lanes group 1 -> [A0] <-> [ 8, 9]
    group1_50G=[1] # A_lanes group 2 -> [A1] <-> {10,11]
    group2_50G=[2] # A_lanes group 3 -> [A2] <-> [12,13]
    group3_50G=[3] # A_lanes group 4 -> [A3] <-> [14,15]
    
    #Determine the corresponding B-Lanes for each group of A-Lanes
    B_lanes=[]
    if all(elem in A_lanes for elem in group0_50G): # If A_lanes contains [0]
        B_lanes+=[ 8, 9]                        
    if all(elem in A_lanes for elem in group1_50G): # If A_lanes contains [1]
        B_lanes+=[10,11]                      
    if all(elem in A_lanes for elem in group2_50G): # If A_lanes contains [2]
        B_lanes+=[12,13]
    if all(elem in A_lanes for elem in group3_50G): # If A_lanes contains [3]
        B_lanes+=[14,15]
    #else:
    #    print("\n*** 50G-1 Gearbox Setup: Invalid Target A-Lanes specified!\n")
    #    return
    
    lanes = sorted(list(set(A_lanes + B_lanes)))
    prbs_mode_select(lane=lanes, prbs_mode='functional')

    if all(elem in A_lanes for elem in group0_50G):  # If A_lanes contains [0]
        fw_config_cmd(config_cmd=0x90b0,config_detail=0x0000) # 0x90b0 = First, FW destroy any instances of these lanes being already used      
    if all(elem in A_lanes for elem in group1_50G):  # If A_lanes contains [1]
        fw_config_cmd(config_cmd=0x90b1,config_detail=0x0000) # 0x90b1 = First, FW destroy any instances of these lanes being already used
    if all(elem in A_lanes for elem in group2_50G): # If A_lanes contains [2]
        fw_config_cmd(config_cmd=0x90b2,config_detail=0x0000) # 0x90b2 = First, FW destroy any instances of these lanes being already used
    if all(elem in A_lanes for elem in group3_50G): # If A_lanes contains [3]
        fw_config_cmd(config_cmd=0x90b3,config_detail=0x0000) # 0x90b3 = First, FW destroy any instances of these lanes being already used

    if all(elem in A_lanes for elem in group0_50G):  # If A_lanes contains [0]
        if fec_b_byp==False:
            if print_en: print("\n...FW Gearbox 50G-1: Lanes A0 to B0-B1 with FEC_A0/FEC_B0..."),
            fw_config_cmd(config_cmd=0x80b0,config_detail=0x0000) # 0x80b0 = FW Activate Gearbox 50G-1 for Lane A0
        else:
            if print_en: print("\n...FW Gearbox 50G-1: Lanes A0 to B0-B1 with FEC_A0 (FEC_B0 Bypassed)..."),
            fw_config_cmd(config_cmd=0x80b8,config_detail=0x0000) # no fec 0x80b8 = FW Activate Gearbox 50G-1 for Lane A0   
   
    if all(elem in A_lanes for elem in group1_50G):  # If A_lanes contains [1]
        if fec_b_byp==False:
            if print_en: print("\n...FW Gearbox 50G-1: Lanes A1 to B2-B3 with FEC_A1/FEC_B1..."),
            fw_config_cmd(config_cmd=0x80b1,config_detail=0x0000) # 0x80b1 = FW Activate Gearbox 50G-1 for Lane A1
        else:
            if print_en: print("\n...FW Gearbox 50G-1: Lanes A1 to B2-B3 with FEC_A1 (FEC_B1 Bypassed)..."),
            fw_config_cmd(config_cmd=0x80b9,config_detail=0x0000) # no fec 0x80b9 = FW Activate Gearbox 50G-1 for Lane A1   
    
    if all(elem in A_lanes for elem in group2_50G):  # If A_lanes contains [2]
        if fec_b_byp==False:
            if print_en: print("\n...FW Gearbox 50G-1: Lanes A2 to B4-B5 with FEC_A0/FEC_B0..."),
            fw_config_cmd(config_cmd=0x80b2,config_detail=0x0000) # 0x80b0 = FW Activate Gearbox 50G-1 for Lane A2
        else:
            if print_en: print("\n...FW Gearbox 50G-1: Lanes A2 to B4-B5 with FEC_A2 (FEC_B2 Bypassed)..."),
            fw_config_cmd(config_cmd=0x80ba,config_detail=0x0000) # no fec 0x80ba = FW Activate Gearbox 50G-1 for Lane A2   
    
    if all(elem in A_lanes for elem in group3_50G):  # If A_lanes contains [3]
        if fec_b_byp==False:
            if print_en: print("\n...FW Gearbox 50G-1: Lanes A3 to B6-B7 with FEC_A3/FEC_B3..."),
            fw_config_cmd(config_cmd=0x80b3,config_detail=0x0000) # 0x80b0 = FW Activate Gearbox 50G-1 for Lane A3
        else:
            if print_en: print("\n...FW Gearbox 50G-1: Lanes A3 to B6-B7 with FEC_A3 (FEC_B3 Bypassed)..."),
            fw_config_cmd(config_cmd=0x80bb,config_detail=0x0000) # no fec 0x80bb = FW Activate Gearbox 50G-1 for Lane A3
    
    if print_en: print("Done!")
    #fec_status()
####################################################################################################
def watch(function,*args):
    import os
    os.system('cls' if os.name == 'nt' else 'clear')
    try:
        while True:            
            print('\033[0;0H') # go to top left corner of screen
            print('\033[f') # go to top left corner of screen
            function(*args)
            time.sleep(1)            
    except KeyboardInterrupt:
        pass
####################################################################################################
def twos_to_int(twos_val, bitWidth):
    '''
    return a signed decimal number
    '''
    mask = 1<<(bitWidth - 1)
    return -(twos_val & mask) + (twos_val & ~mask)
    
####################################################################################################
def int_to_twos(value, bitWidth):
    '''
    return a twos complement number
    '''
    return (2**bitWidth)+value if value < 0 else value
####################################################################################################
def dec2bin(x):
  x -= int(x)
  bins = []
  for i in range(8):
    x *= 2
    bins.append(1 if x>=1. else 0)
    x -= int(x)
    #print bins
  value = 0
  for a in range(8):
    value = value+ bins[7-a]*pow(2,a)
    
  return value
####################################################################################################
# 
# Gray Code to Binary Conversion
####################################################################################################
def Bin_Gray(bb=0):
    gg=bb^(bb>>1)
    return gg

####################################################################################################
def Gray_Bin(gg=0): # up to 7-bit Gray Number
    bb1=(gg&0x40)
    bb2=(gg^(bb1>>1))&(0x20)
    bb3=(gg^(bb2>>1))&(0x10)
    bb4=(gg^(bb3>>1))&(0x8)
    bb5=(gg^(bb4>>1))&(0x4)
    bb6=(gg^(bb5>>1))&(0x2)
    bb7=(gg^(bb6>>1))&(0x1)
    bb=bb1+bb2+bb3+bb4+bb5+bb6+bb7
    return bb
####################################################################################################
def pol (tx_pol=None, rx_pol=None, lane=None, print_en=1):

    lanes = get_lane_list(lane)    
    Slice=gSlice
    #get_lane_mode('all')
    result={}
           
    for ln in lanes:
        get_lane_mode(ln)
        c = Pam4Reg if lane_mode_list[ln] == 'pam4' else NrzReg
        if (tx_pol!=None):
            wreg(c.tx_pol_addr, tx_pol,ln)
        if (rx_pol!=None):
            wreg(c.rx_pol_addr, rx_pol,ln)

        tx_pol_this_lane= rreg(c.tx_pol_addr,ln)
        rx_pol_this_lane= rreg(c.rx_pol_addr,ln)
        result[ln] = tx_pol_this_lane, rx_pol_this_lane
        #if(print_en): print("\nSlice %d Lane %s (%4s) Polarity: TX: %d -- RX: %d"%(Slice,lane_name_list[ln],gEncodingMode[gSlice][ln][0].upper(),tx_pol_this_lane, rx_pol_this_lane)),

    if print_en: # Print Status
        get_lane_mode('all')
        print("\nSlice %d, Lane:"%(sel_slice())),
        for ln in range(len(lane_name_list)):
            print(" %2s" %(lane_name_list[ln])),
        print("\n  TX Polarity:"),
        for ln in range(len(lane_name_list)):
            c = Pam4Reg if lane_mode_list[ln] == 'pam4' else NrzReg
            print(" %2d"  %(rreg(c.tx_pol_addr,ln))),
        print("\n  RX Polarity:"),
        for ln in range(len(lane_name_list)):
            c = Pam4Reg if lane_mode_list[ln] == 'pam4' else NrzReg
            print(" %2d" %(rreg(c.rx_pol_addr,ln))),
    else:
        return tx_pol_this_lane, rx_pol_this_lane
####################################################################################################
def gc (tx_gc=None, rx_gc=None, lane=None, print_en=None):

    lanes = get_lane_list(lane)
        
    Slice=gSlice
    #get_lane_mode('all')
    
    for ln in lanes:
        get_lane_mode(ln)
        if gEncodingMode[gSlice][ln][0] == 'nrz': c=NrzReg
        else:                           c=Pam4Reg

        if (tx_gc!=None and rx_gc==None):
            wreg(c.tx_gray_en_addr, tx_gc,ln)
            if gEncodingMode[gSlice][ln][0] == 'pam4': wreg(c.rx_gray_en_addr, tx_gc,ln)
        elif (tx_gc!=None and rx_gc!=None):
            wreg(c.tx_gray_en_addr, tx_gc,ln)
            if gEncodingMode[gSlice][ln][0] == 'pam4': wreg(c.rx_gray_en_addr, rx_gc,ln)
        else:
            if print_en==None: print_en=1 # if no arguments, readout the current polarity settings

        tx_gc_this_lane= rreg(c.tx_gray_en_addr,ln)
        if gEncodingMode[gSlice][ln][0] == 'pam4': rx_gc_this_lane= rreg(c.rx_gray_en_addr,ln)
        else: rx_gc_this_lane=0
        
        if(print_en): print("\nSlice %d Lane %s (%4s) GrayCode: TX: %d -- RX: %d"%(Slice,lane_name_list[ln],gEncodingMode[gSlice][ln][0].upper(),tx_gc_this_lane, rx_gc_this_lane)),
    if(print_en==0): return tx_gc_this_lane, rx_gc_this_lane
####################################################################################################
def pc (tx_pc=None, rx_pc=None, lane=None, print_en=None):

    lanes = get_lane_list(lane)
        
    Slice=gSlice
    #get_lane_mode('all')
    
    for ln in lanes:
        get_lane_mode(ln)
        if gEncodingMode[gSlice][ln][0] == 'nrz': c=NrzReg
        else:                           c=Pam4Reg

        if (tx_pc!=None and rx_pc==None):
            wreg(c.tx_precoder_en_addr, tx_pc,ln)
            if gEncodingMode[gSlice][ln][0] == 'pam4': wreg(c.rx_precoder_en_addr, tx_pc,ln)
        elif (tx_pc!=None and rx_pc!=None):
            wreg(c.tx_precoder_en_addr, tx_pc,ln)
            if gEncodingMode[gSlice][ln][0] == 'pam4': wreg(c.rx_precoder_en_addr, rx_pc,ln)
        else:
            if print_en==None: print_en=1 # if no arguments, readout the current polarity settings

        tx_pc_this_lane= rreg(c.tx_precoder_en_addr,ln)
        if gEncodingMode[gSlice][ln][0] == 'pam4': rx_pc_this_lane= rreg(c.rx_precoder_en_addr,ln)
        else: rx_pc_this_lane= 0
        
        if(print_en): print("\nSlice %d Lane %s (%4s) Precoder: TX: %d -- RX: %d"%(Slice,lane_name_list[ln],gEncodingMode[gSlice][ln][0].upper(),tx_pc_this_lane, rx_pc_this_lane)),
    if(print_en==0): return tx_pc_this_lane, rx_pc_this_lane
####################################################################################################
def msblsb (tx_msblsb=None, rx_msblsb=None, lane=None, print_en=None):

    lanes = get_lane_list(lane)

    Slice=gSlice
    #get_lane_mode('all')
    
    for ln in lanes:
        get_lane_mode(ln)
        if gEncodingMode[gSlice][ln][0] == 'nrz': c=NrzReg
        else:                           c=Pam4Reg

        if (tx_msblsb!=None and rx_msblsb==None):
            wreg(c.tx_msb_swap_addr, tx_msblsb,ln)
            if gEncodingMode[gSlice][ln][0] == 'pam4': wreg(c.rx_msb_swap_addr, tx_msblsb,ln)
        elif (tx_msblsb!=None and rx_msblsb!=None):
            wreg(c.tx_msb_swap_addr, tx_msblsb,ln)
            if gEncodingMode[gSlice][ln][0] == 'pam4': wreg(c.rx_msb_swap_addr, rx_msblsb,ln)
        else:
            if print_en==None: print_en=1 # if no arguments, readout the current polarity settings

        tx_msblsb_this_lane= rreg(c.tx_msb_swap_addr,ln)
        if gEncodingMode[gSlice][ln][0] == 'pam4': rx_msblsb_this_lane= rreg(c.rx_msb_swap_addr,ln)
        else: rx_msblsb_this_lane =0
        
        if(print_en): print("\nSlice %d Lane %s (%4s) MSB-LSB Swap: TX: %d -- RX: %d"%(gSlice,lane_name_list[ln],gEncodingMode[gSlice][ln][0].upper(),tx_msblsb_this_lane, rx_msblsb_this_lane)),
   
    if(print_en==0): return tx_msblsb_this_lane, rx_msblsb_this_lane

####################################################################################################
# reset_lane_pll ()
#
# Power Down/Up (Toggle PU bits of) TXPLL or RXPLL 
# Meanwhile, toggle FRAC_EN of each PLL
#
#################################################################################################### 
def reset_lane_pll (tgt_pll='both', lane=None):
   
    lanes = get_lane_list(lane)      # determine lanes to work on
    #get_lane_mode(lanes)         # Get current PAM4/NRZ modes settings of the specified lane(s)

    
    for ln in lanes:
        get_lane_mode(ln)
        c = Pam4Reg if lane_mode_list[ln].lower() == 'pam4' else NrzReg
  
        tx_frac_en = rreg(c.tx_pll_frac_en_addr, ln)# save, 0x0D7   [13] TX PLL FRAC_EN=x
        rx_frac_en = rreg(c.rx_pll_frac_en_addr, ln)# save, 0x1F0   [13] RX PLL FRAC_EN=y

        wreg(c.rx_pll_pu_addr,               0, ln) # Power down RX PLL while prgramming PLL
        wreg(c.tx_pll_pu_addr,               0, ln) # Power down RX PLL while prgramming PLL
                                          
        wreg(c.tx_pll_frac_en_addr,          0, ln) #  0x0D7   [13] TX PLL FRAC_EN=0
        wreg(c.rx_pll_frac_en_addr,          0, ln) #  0x1F0   [13] RX PLL FRAC_EN=0

        wreg(c.tx_pll_frac_en_addr,          1, ln) #  0x0D7   [13] TX PLL FRAC_EN=1
        wreg(c.rx_pll_frac_en_addr,          1, ln) #  0x1F0   [13] RX PLL FRAC_EN=1
                                          
        wreg(c.tx_pll_frac_en_addr,          0, ln) #  0x0D7   [13] TX PLL FRAC_EN=0
        wreg(c.rx_pll_frac_en_addr,          0, ln) #  0x1F0   [13] RX PLL FRAC_EN=0
                                          
        wreg(c.rx_pll_pu_addr,               1, ln) # Power up RX PLL after toggling FRAC_EN
        wreg(c.tx_pll_pu_addr,               1, ln) # Power up TX PLL after toggling FRAC_EN

        #### Enable Fractional PLLs after programming PLLs, if needed
        wreg(c.tx_pll_frac_en_addr, tx_frac_en, ln) # restore 0x0D7   [13] TX PLL FRAC_EN=x
        wreg(c.rx_pll_frac_en_addr, rx_frac_en, ln) # restore 0x1F0   [13] RX PLL FRAC_EN=y

####################################################################################################
# set/get pll caps
#
#
#################################################################################################### 
def pll_cap_orig (tx_cap=None, rx_cap=None, lane=None, print_en=1):
   
    lanes = get_lane_list(lane)      # determine lanes to work on
    result = {}

    if(print_en): print("\n+-------------------------------+"),
    top_pll_A_cap = rreg([0x9501,[12,6]])
    top_pll_B_cap = rreg([0x9601,[12,6]])
    if(print_en): print("\n| Dev%d TopPLL A/B |  %3d   %3d  |"%(gSlice, top_pll_A_cap, top_pll_B_cap)),
    if(print_en): print("\n+-------------------------------+"),
    if(print_en): print("\n| Lane | DataRate | TxCap RxCap |"),
    if(print_en): print("\n+-------------------------------+"),

    pll_params = get_lane_pll(lanes)

    for ln in lanes:
        if(tx_cap!=None): wreg(c.tx_pll_lvcocap_addr, tx_cap,ln)
        if(rx_cap!=None): wreg(c.rx_pll_lvcocap_addr, rx_cap,ln)
        tx_cap_this_lane= rreg(c.tx_pll_lvcocap_addr,ln)
        rx_cap_this_lane= rreg(c.rx_pll_lvcocap_addr,ln)
        result[ln] = [tx_cap_this_lane,rx_cap_this_lane]
        pll_params = get_lane_pll(lanes)
        fvco=pll_params[ln][0][0]
        if(print_en): print("\n|  %2s  | %8.5f |  %3d   %3d  |"%(lane_name_list[ln],fvco,tx_cap_this_lane, rx_cap_this_lane)),
        if(print_en) and (ln==lanes[-1] or ln==7):
            print("\n+-------------------------------+"),
    if(print_en==0): return result
####################################################################################################
# set/get pll caps
#
#
#################################################################################################### 
def pll_cap (tx_cap=None, rx_cap=None, lane=None, print_en=1):
   
    lanes = get_lane_list(lane)      # determine lanes to work on
    result = {}

    if(print_en): print("\n+-------------------------------------------------------------------+"),
    top_pll_A_cap = rreg([0x9501,[12,6]])
    top_pll_B_cap = rreg([0x9601,[12,6]])
    if(print_en): print("\n| Dev%d TopPLL A/B |     |  A-cap: %3d         |   B-cap: %3d        |"%(gSlice, top_pll_A_cap, top_pll_B_cap)),
    if(print_en): print("\n+-------------------------------------------------------------------+"),
    if(print_en): print("\n|      |          | Cal |      TX PLL Cal     |     RX PLL Cal      |"),
    if(print_en): print("\n| Lane | DataRate | Done| Min  Wr/Rd/Curr Max | Min  Wr/Rd/Curr Max |"),
    if(print_en): print("\n+-------------------------------------------------------------------+"),

    pll_params = get_lane_pll(lanes)

    for ln in lanes:
        pll_cal_done= (fw_debug_cmd(0,5,ln)>>ln)&1 

        tx_cap_min = fw_debug_cmd(0,10,ln)
        tx_cap_max = fw_debug_cmd(0,11,ln)
        rx_cap_min = fw_debug_cmd(0,12,ln)
        rx_cap_max = fw_debug_cmd(0,13,ln)
        tx_cap_write = fw_debug_cmd(0,14,ln)
        tx_cap_read  = fw_debug_cmd(0,15,ln)
        rx_cap_write = fw_debug_cmd(0,16,ln)
        rx_cap_read  = fw_debug_cmd(0,17,ln)

        if(tx_cap!=None): wreg(c.tx_pll_lvcocap_addr, tx_cap,ln)
        if(rx_cap!=None): wreg(c.rx_pll_lvcocap_addr, rx_cap,ln)
        tx_cap_this_lane= rreg(c.tx_pll_lvcocap_addr,ln)
        rx_cap_this_lane= rreg(c.rx_pll_lvcocap_addr,ln)
        result[ln] = [tx_cap_min,tx_cap_this_lane,tx_cap_max, rx_cap_min, rx_cap_this_lane,rx_cap_max]
        pll_params = get_lane_pll(lanes)
        fvco=pll_params[ln][0][0]
        if(print_en): print("\n|  %2s  | %8.5f | %2d  | %3d  %2d/%2d/%2d   %2d  | %3d  %2d/%2d/%2d   %2d  |"%(lane_name_list[ln],fvco,pll_cal_done,tx_cap_min,tx_cap_write, tx_cap_read,tx_cap_this_lane,tx_cap_max, rx_cap_min, rx_cap_write, rx_cap_read,rx_cap_this_lane,rx_cap_max)),
        if(print_en) and (ln==lanes[-1] or ln==7):
            print("\n+-------------------------------------------------------------------+"),

    if(print_en==0): return result
####################################################################################################
# pll cal for TOP PLL  (REFCLK Source)
#
# sweeps pll cap and checks pll lock
#################################################################################################### 
def pll_cal_top0( side = 'A'):
    window=0x3fff
    toppll_lock = 0
    if side.upper() == 'A':
        Base_addr = 0x9500
    else:
        Base_addr = 0x9600
    out = 0
    inside = 0
    f2 = open('pll_cal_top_log.txt','w')
    wregBits(Base_addr + 0x01,[12,6],0,16)
    wregBits(Base_addr + 0x0D,[14,0],window,16)
    setting = rregBits(Base_addr + 0x0D,[14,0],16) # set counter to the reference frequency
    while 1:
        wregBits(Base_addr + 0x12, [3], 0,16)  # PD_CAL_FCAL = 0  
        wregBits(Base_addr + 0x10, [7], 0,16) # LO_OPEN = 0 
        wregBits(Base_addr + 0x0D, [15], 0,16) # fcal_start = 0  
        wregBits(Base_addr + 0x0D, [15], 1,16) # fcal_start = 1  
        while(rregBits(Base_addr + 0x0F, [15],16) == 0):  #check fcal_done. If it's done, continue to the next
            pass
        readout = rregBits(Base_addr + 0x0E,[15,0],16)  # read out the counter value(fcal_cnt_op[15:0]).
        vcocap = rregBits(Base_addr + 0x01,[12,6],16)
        if(abs(readout-setting) > 2):  
            if (vcocap == 0x7f):
                vcocap_min = 0
                toppll_lock = 1
                print >> f2, "The frequency setting is out of pll range(min)"
                break
            else:
                vcocap = rregBits(Base_addr + 0x01,[12,6],16) + 0x1
                wregBits(Base_addr + 0x01, [12,6], vcocap,16)
        else:
            vcocap_min = rregBits(Base_addr + 0x01,[12,6],16)
            print >> f2,  "Min vcocap = %s" % (bin(vcocap_min))
            break

    wregBits(Base_addr + 0x01,[12,6],0x7f,16)
    wregBits(Base_addr + 0x0D,[14,0],window,16)
    while 1:
        wregBits(Base_addr + 0x12, [3], 0,16)
        wregBits(Base_addr + 0x10, [7], 0,16)
        wregBits(Base_addr + 0x0D, [15], 0,16)
        wregBits(Base_addr + 0x0D, [15], 1,16)
        while(rregBits(Base_addr + 0x0F, [15],16) == 0):
            pass

        readout = rregBits(Base_addr + 0x0E,[15,0],16)
        vcocap = rregBits(Base_addr + 0x01,[12,6],16)
        if(abs(readout-window) > 2):
            if (vcocap == 0x0):
                vcocap_max = 0
                toppll_lock = 1
                print >> f2, "The frequency setting is out of pll range(max)"
                break
            else:
                vcocap = rregBits(Base_addr + 0x01,[12,6],16) - 0x1
                wregBits(Base_addr + 0x01, [12,6], vcocap,16)
        else:
            vcocap_max = rregBits(Base_addr + 0x01,[12,6],16)
            print >> f2,  "Max vcocap = %s" % (bin(vcocap_max))
            break
           
    new_vcocap = int((vcocap_max + vcocap_min)/2)
    if ((vcocap_max == 0x7f) & ((vcocap_max-vcocap_min)<4)):
        new_vcocap = vcocap_max
    else:
        pass
    if ((vcocap_min == 0x00) & ((vcocap_max-vcocap_min)<4)):
        new_vcocap = vcocap_min
    else:
        pass
    print >> f2, "new_vcocap = %s" % (new_vcocap)
    print >> f2, "Min CAP value : %s" % (vcocap_min)
    print >> f2, "Max CAP value : %s" % (vcocap_max)
    wregBits(Base_addr + 0x01, [12,6], new_vcocap,16)
    print "CAP value : %s" % (hex(rregBits(Base_addr + 0x01, [12,6])))
    f2.close()
    return new_vcocap
####################################################################################################
# pll cal for TOP PLL  (REFCLK Source)
#
# sweeps pll cap and checks pll lock
#################################################################################################### 
def pll_cal_top1(tgt_pll=None, lane=None, print_en=1):
    
    if tgt_pll==None: tgt_pll='both' # set both top PLLs, A and B
    pll_list=['A','B'] if tgt_pll=='both' else tgt_pll.upper()

    result = {}
    pll_cal_result=[[1,2,3],[4,5,6]] # TX_PLL[vcocap_min,new_vcocap,vcocap_max], RX_PLL[vcocap_min,new_vcocap,vcocap_max],
    log_en=1
    window=0x2000
    out = 0
    inside = 0
    
    if(log_en): f2 = open('pll_cal_TOP_log.txt','w')
    if print_en: print "\n     -- Top PLL Cap --",
    if print_en: print "\nPLL, Min, Old/New, Max,",

    for top_pll_side in pll_list:
        if print_en: print "\n%3s,"%top_pll_side,
        if top_pll_side == 'A':
            Base_addr = 0x9500
            top_pll_index=0
        else:
            Base_addr = 0x9600
            top_pll_index=1

        toppll_lock = 0
        
        if(log_en): print >> f2, "TopPLL  ,Dir, Cap, Target, ReadOut, Delta"
        orig_vcocap = rregBits(Base_addr + 0x01,[12,6],16)  # original cap value
        
        ### Sweep upward and find min vcocap
        wregBits(Base_addr + 0x01,[12,6],0,16)
        wregBits(Base_addr + 0x0D,[14,0],window,16)
        target_cnt = rregBits(Base_addr + 0x0D,[14,0],16) # set counter to the reference frequency
        while 1:
            wregBits(Base_addr + 0x12, [3], 0,16)  # PD_CAL_FCAL = 0  
            wregBits(Base_addr + 0x10, [7], 0,16) # LO_OPEN = 0 
            wregBits(Base_addr + 0x0D, [15], 0,16) # fcal_start = 0  
            wregBits(Base_addr + 0x0D, [15], 1,16) # fcal_start = 1  
            while(rregBits(Base_addr + 0x0F, [15],16) == 0):  #check fcal_done. If it's done, continue to the next
                pass
            readout = rregBits(Base_addr + 0x0E,[15,0],16)  # read out the counter value(fcal_cnt_op[15:0]).
            vcocap = rregBits(Base_addr + 0x01,[12,6],16)
            if(log_en): print >> f2, "TopPLL-%s, Up, %3d,   %04X,   %04X,   %04X"%(top_pll_side, vcocap,target_cnt,readout, target_cnt-readout)
            if(abs(readout-target_cnt) > 2):  
                if (vcocap == 0x7f):
                    vcocap_min = 0
                    toppll_lock = 1
                    if(log_en): print >> f2, "Top PLL Side %s,  UP: The frequency target_cnt is out of pll range(min)"%top_pll_side
                    break
                else:
                    vcocap = rregBits(Base_addr + 0x01,[12,6],16) + 0x1
                    wregBits(Base_addr + 0x01, [12,6], vcocap,16)
            else:
                vcocap_min = rregBits(Base_addr + 0x01,[12,6],16)
                #if(log_en): print >> f2,  "Min vcocap = %d" % (vcocap_min)
                break

        ### Sweep downward and find max vcocap
        wregBits(Base_addr + 0x01,[12,6],0x7f,16)
        wregBits(Base_addr + 0x0D,[14,0],window,16)
        while 1:
            wregBits(Base_addr + 0x12, [3], 0,16)
            wregBits(Base_addr + 0x10, [7], 0,16)
            wregBits(Base_addr + 0x0D, [15], 0,16)
            # fcal start pulse delay
            wregBits(Base_addr + 0x0D, [15], 1,16)
            # fcal done check delay
            while(rregBits(Base_addr + 0x0F, [15],16) == 0):
                pass

            readout = rregBits(Base_addr + 0x0E,[15,0],16)
            vcocap = rregBits(Base_addr + 0x01,[12,6],16)
            if(log_en): print >> f2, "TopPLL-%s, Dn, %3d,   %04X,   %04X,   %04X"%(top_pll_side, vcocap,target_cnt,readout, target_cnt-readout)
            if(abs(readout-target_cnt) > 2):
                if (vcocap == 0x0):
                    vcocap_max = 0
                    toppll_lock = 1
                    if(log_en): print >> f2, "Top PLL Side %s,  DN: The frequency target_cnt is out of pll range(min)"%top_pll_side
                    break
                else:
                    vcocap = rregBits(Base_addr + 0x01,[12,6],16) - 0x1
                    wregBits(Base_addr + 0x01, [12,6], vcocap,16)
            else:
                vcocap_max = rregBits(Base_addr + 0x01,[12,6],16)
                #if(log_en): print >> f2,  "Max vcocap = %d" % (vcocap_max)
                break
               
        ### Find the optimum cap from min and max
        if (vcocap_max == 0x7f) and (vcocap_min == 0x00): # Search FAILED
            new_vcocap = orig_vcocap
        else: # Search SUCCESSFUL
            new_vcocap = int((vcocap_max + vcocap_min)/2) 
            if   ((vcocap_max == 0x7f) and ((vcocap_max-vcocap_min)<4)):
                new_vcocap = vcocap_max
            elif ((vcocap_min == 0x00) and ((vcocap_max-vcocap_min)<4)):
                new_vcocap = vcocap_min

        wregBits(Base_addr + 0x01, [12,6], new_vcocap,16)
        pll_cal_result[top_pll_index]=[vcocap_min,new_vcocap,vcocap_max]
        flag= '>' if orig_vcocap!=new_vcocap else ','
        if print_en: print "%3d, %3d%s%3d, %3d, " % (vcocap_min,orig_vcocap,flag,new_vcocap,vcocap_max),
        if(log_en): print >> f2,"\nTop PLL %s VCOCAPS: Min/Center(Orig)/Max: %2d / %2d(%2d) / %2d\n" % (top_pll_side,vcocap_min,new_vcocap,orig_vcocap,vcocap_max)

        wregBits(Base_addr + 0x10,  [7],  0,16) # LO_OPEN = 0 
        wregBits(Base_addr + 0x0D,  [15], 0,16) # fcal_start = 0  
        wregBits(Base_addr + 0x0D,[14,0], 0,16) # facl window=0
        wregBits(Base_addr + 0x12,   [3], 1,16) # PD_CAL_FCAL = 1
        #### End of this Top PLL      
        
    #### End of both Top PLL
    result=[pll_cal_result[0],pll_cal_result[1]]  # [0]=A and [1]=B
    if print_en: print "\n"
    if(log_en): f2.close()
    if print_en==0: return result

####################################################################################################  
def pll_cal_top(side ='both'): 

    if side.upper() == 'A' or side.upper() == 'BOTH':
        Base_addr = 0x9500
        capA = pll_cal_top3(base=Base_addr)
    if side.upper() == 'B' or side.upper() == 'BOTH':
        Base_addr = 0x9600
        capB = pll_cal_top3(base=Base_addr)

    if side.upper() == 'A':
        return capA
    if side.upper() == 'B':
        return capB
    if side.upper() == 'BOTH':  
        return [capA,capB]

####################################################################################################
# pll cal for TOP PLL  (REFCLK Source)
#
# sweeps pll cap and checks pll lock
####################################################################################################  
def pll_cal_top3(window=0x3fff, base=0x9500):

    wregBits(base+0x10, [6, 4], 4)
    wregBits(base+0x10, [7], 1)        
    wregBits(base+0xd, [14, 0], window)
    wregBits(base+0x12, [3], 0)
    wregBits(base+0x0d, [15], 0)
    vco_min = 0
    vco_max = 0
    no_cross = True
    
    for vco in range(0, 128, 2):
        wregBits(base+1, [12, 6], vco)        
        wregBits(base+0x0d, [15], 1)
        count=0
        while rregBits(base+0xf, [15])==0 and count<1000: count+=1
        readout = rregBits(base+0xe, [15, 0])
        wregBits(base+0xd, [15], 0)
        if (count>=1000):
            print "cal_done timeout at %d", vco
            continue
        else:
            if readout > window :
                vco_min = vco
            elif no_cross :
                vco_max = vco
                no_cross = False          
            #print "scan, %3d, %04x, %04x" % (vco, window, readout)

    vco = int((vco_min+vco_max)/2)
    wregBits(base+1, [12, 6], vco)
    wregBits(base+0x10, [7], 0)
    wregBits(base+0xd, [14, 0], 0)
    wregBits(base+0x12, [3], 1)
    #print "Top PLL %04X final vco = %3d" % (base,vco)    
    return vco

####################################################################################################
# pll cal for TOP PLL  (REFCLK Source)
#
# sweeps pll cap and checks pll lock
####################################################################################################     
def pll_cal_top2(lane=0, window=0x3fff, openLoop=False, scanAll=None, dir_down=False):
    # Prolog
    base=0x9500+lane*0x100
    
    if openLoop:
        wregBits(base+0x10, [6, 4], 4)
        wregBits(base+0x10, [7], 1)
    else:
        wregBits(base+0x10, [7], 0)
        
    wregBits(base+0xd, [14, 0], window)
    wregBits(base+0x12, [3], 0)
    wregBits(base+0x0d, [15], 0)
    vco_max = 128
    if scanAll is not None:
        if not isinstance(scanAll, list):
            scanAll=range(vco_max)
        for vco in scanAll:
            if dir_down :
                wregBits(base+1, [12, 6], vco)
            else :
                wregBits(base+1, [12, 6], vco_max-vco)           
            wregBits(base+0x0d, [15], 1)
            count=0
            while rregBits(base+0xf, [15])==0 and count<1000: count+=1
            readout = rregBits(base+0xe, [15, 0])
            wregBits(base+0xd, [15], 0)
            if (count>=1000):
                print "cal_done timeout at %d", vco
            else:
                print "lane %d, scan, %3d, %04x, %04x" % (lane, vco, window, readout)
        return
    vco=0
    while vco<0x80:
        wregBits(base+1, [12, 6], vco)
        wregBits(base+0x0d, [15], 1)
        count=0
        while rregBits(base+0xf, [15])==0 and count<1000: count+=1
        readout = rregBits(base+0xe, [15, 0])
        wregBits(base+0x0d, [15], 0)
        if (count>=1000):
            print "cal_done timeout at %d", vco
            return
        print "lane %d, up, %3d, %04x, %04x" % (lane, vco, window, readout)
        if abs(readout-window)>2:
            if (vco==0x7f):
                vco_min = 0
                print "top pll scan up no lock"
                return
            else:
                vco+=1
                continue
        else:
            vcomin = vco
            print "lane %d vcomin = %3d" % (lane, vcomin)
            break
    vco=0x7f
    while vco>=0:
        wregBits(base+1, [12, 6], vco)
        wregBits(base+0xd, [15], 1)
        count=0
        while rregBits(base+0xf, [15])==0 and count<1000: count+=1
        readout = rregBits(base+0xe, [15, 0])
        wregBits(base+0xd, [15], 0)
        if (count>=1000):
            print "cal_done timeout at %d", vco
            return
        print "lane %d, down, %3d, %04x, %04x" % (lane, vco, window, readout)
        if abs(readout-window)>2:
            if (vco==0):
                print "top pll scan down no lock"
                return
            else:
                vco-=1
                continue
        else:
            vcomax = vco
            print "lane %d vcomax = %3d" % (lane, vcomax)
            break
    vco = int((vcomin+vcomax)/2)
    wregBits(base+1, [12, 6], vco)
    # Epilog
    wregBits(base+0x10, [7], 0)
    wregBits(base+0xd, [14, 0], 0)
    wregBits(base+0x12, [3], 1)
    print "lane %d final vco = %3d" % (lane, vco)

####################################################################################################
# pll cal for RX PLL, TX PLL, or both 
#
# sweeps pll cap and checks pll lock
#################################################################################################### 
def pll_cal(tgt_pll=None, lane=None, print_en=1):

    if tgt_pll==None: tgt_pll='both' # set both PLLs with datarate passed
    pll_list=['tx','rx'] if tgt_pll=='both' else tgt_pll

    lanes = get_lane_list(lane)
    result = {}
    pll_cal_result=[[1,2,3],[4,5,6]] # TX_PLL[vcocap_min,new_vcocap,vcocap_max], RX_PLL[vcocap_min,new_vcocap,vcocap_max],
    log_en=0
    window=0x2000
    out = 0
    inside = 0
    rx_lock = 0
       
    if(log_en): f2 = open('pll_cal_log.txt','w')
    if print_en: print "\n    --- TX PLL Cap ---  --- RX PLL Cap ---",
    if print_en: print "\nLn, Min, Old/New, Max,  Min, Old/New, Max",

    for ln in lanes:
        if print_en: print "\n%s,"%lane_name_list[ln],
        for lane_pll in pll_list:
            index=-1
            if 'tx' in lane_pll: # TX PLL CAL
                index=0
                vcocap_addr      = [0xDB, [14,8]]
                fcal_window_addr = [0xD1, [14,0]]
                fcal_cnt_op_addr = [0xD0, [15,0]]
                fcal_pd_cal_addr = [0xDB,    [6]]
                fcal_lo_open_addr= [0xD9,   [11]]
                fcal_start_addr  = [0xD1,   [15]]
                fcal_done_addr   = [0xCF,   [15]]
            else: # RX PLL CAL 
                index=1
                vcocap_addr      = [0x1F5,[15,9]]
                fcal_window_addr = [0x1EA,[14,0]]
                fcal_cnt_op_addr = [0x1E9,[15,0]]
                fcal_pd_cal_addr = [0x1F5,   [6]]
                fcal_lo_open_addr= [0x1F3,  [11]]
                fcal_start_addr  = [0x1EA,  [15]]
                fcal_done_addr   = [0x1E8,  [15]]

            
            if(log_en): print >> f2, "Lane,Dir, Cap, Target, ReadOut"          
            orig_vcocap = rreg(vcocap_addr,ln)  # original cap value

            ### Sweep upward and find min vcocap
            wreg(vcocap_addr,0x0,ln)
            wreg(fcal_window_addr,window,ln)
            target_cnt = rreg(fcal_window_addr,ln) # set counter to the reference frequency
            while 1:
                wreg(fcal_pd_cal_addr, 0,ln)  # PD_CAL_FCAL = 0  
                wreg(fcal_lo_open_addr, 0,ln) # LO_OPEN = 0 
                wreg(fcal_start_addr, 0,ln) # fcal_start = 0  
                wreg(fcal_start_addr, 1,ln) # fcal_start = 1 
                
                while(rreg(fcal_done_addr,ln) == 0):  #check fcal_done. If it's done, continue to the next
                    pass
                readout = rreg(fcal_cnt_op_addr,ln)  # read out the counter value(fcal_cnt_op[15:0]).
                vcocap = rreg(vcocap_addr,ln)
                if(log_en): print >> f2, "%4d, Up, %3d,    %04X,   %04X"%(ln, vcocap,target_cnt,readout)
                if(abs(readout-target_cnt) > 2):  
                    if (vcocap == 0x7f):
                        vcocap_min = 0
                        rx_lock = 1
                        if(log_en): print >> f2, "Lane %d,  UP: The frequency target_cnt is out of pll range(min)"%ln
                        break
                    else:
                        vcocap = rreg(vcocap_addr,ln) + 0x1
                        wreg(vcocap_addr, vcocap,ln)
                else:
                    vcocap_min = rreg(vcocap_addr,ln)
                    #if(log_en): print >> f2,  "Min vcocap = %d" % (vcocap_min)
                    break

            ### Sweep downward and find max vcocap
            wreg(vcocap_addr,0x7f,ln)
            wreg(fcal_window_addr,window,ln)
            while 1:
                wreg(fcal_pd_cal_addr, 0,ln)
                wreg(fcal_lo_open_addr, 0,ln)
                wreg(fcal_start_addr, 0,ln)
                # fcal start pulse delay
                wreg(fcal_start_addr, 1,ln)
                # fcal done check delay
                while(rreg(fcal_done_addr,ln) == 0):
                    pass

                readout = rreg(fcal_cnt_op_addr,ln)
                vcocap = rreg(vcocap_addr,ln)
                if(log_en): print >> f2, "%4d, Dn, %3d,    %04X,   %04X"%(ln, vcocap,target_cnt,readout)

                if(abs(readout-target_cnt) > 2):
                    if (vcocap == 0x0):
                        vcocap_max = 0
                        rx_lock = 1
                        if(log_en): print >> f2, "Lane %d,Down: The frequency target_cnt is out of pll range(max)"%ln
                        break
                    else:
                        vcocap = rreg(vcocap_addr,ln) - 0x1
                        wreg(vcocap_addr, vcocap,ln)
                else:
                    vcocap_max = rreg(vcocap_addr,ln)
                    #if(log_en): print >> f2,  "Max vcocap = %d" % (vcocap_max)
                    break
                    
            ### Find the optimum cap from min and max
            if (vcocap_max == 0x7f) and (vcocap_min == 0x00): # Search FAILED
                new_vcocap = orig_vcocap
            else: # Search SUCCESSFUL
                new_vcocap = int((vcocap_max + vcocap_min)/2) 
                if   ((vcocap_max == 0x7f) and ((vcocap_max-vcocap_min)<4)):
                    new_vcocap = vcocap_max
                elif ((vcocap_min == 0x00) and ((vcocap_max-vcocap_min)<4)):
                    new_vcocap = vcocap_min

            wreg(vcocap_addr, new_vcocap,ln)
            pll_cal_result[index]=[vcocap_min,new_vcocap,vcocap_max]
            flag= '>' if orig_vcocap!=new_vcocap else ','
            if print_en: print "%3d, %3d%s%3d, %3d, " % (vcocap_min,orig_vcocap,flag,new_vcocap,vcocap_max),
            if(log_en): print >> f2,"\nLane %2d %s VCOCAPS: Min/Center(Orig)/Max: %2d / %2d(%2d) / %2d\n" % (ln,lane_pll,vcocap_min,new_vcocap,orig_vcocap,vcocap_max)
            
            ### Disable TX or RX PLL CAL circuit after done with this Lane's PLL
            wreg(fcal_lo_open_addr, 0,ln)
            wreg(fcal_start_addr,   0,ln)
            wreg(fcal_window_addr,  0,ln)
            wreg(fcal_pd_cal_addr,  1,ln)
            ### end of this PLL, TX or RX
            
        result[ln]=[pll_cal_result[0],pll_cal_result[1]]
        #### End of this lane
    
    if print_en: print "\n"
    if(log_en): f2.close()
    if print_en==0: return result
def pll_cal3(window=0x3fff, base=0x7000, lane_pll='tx'):
    if 'tx' in lane_pll: # TX PLL CAL
        vcocap_addr      = [0xDB, [14,8]]
        fcal_window_addr = [0xD1, [14,0]]
        fcal_vctrl_loopen_sel_addr = [0xD9,[10,8]]
        fcal_cnt_op_addr = [0xD0, [15,0]]
        fcal_pd_cal_addr = [0xDB,    [6]]
        fcal_lo_open_addr= [0xD9,   [11]]
        fcal_start_addr  = [0xD1,   [15]]
        fcal_done_addr   = [0xCF,   [15]]
    else: # RX PLL CAL 
        index=1
        vcocap_addr      = [0x1F5,[15,9]]
        fcal_window_addr = [0x1EA,[14,0]]
        fcal_vctrl_loopen_sel_addr = [0x1F3,[10,8]]
        fcal_cnt_op_addr = [0x1E9,[15,0]]
        fcal_pd_cal_addr = [0x1F5,   [6]]
        fcal_lo_open_addr= [0x1F3,  [11]]
        fcal_start_addr  = [0x1EA,  [15]]
        fcal_done_addr   = [0x1E8,  [15]]
        
    wreg(fcal_vctrl_loopen_sel_addr, 4) #VCTRL_LOOPEN_SEL[2:0]
    wreg(fcal_lo_open_addr, 1)     # LO_OPEN  
    wreg(fcal_window_addr, window) # fcal_timer_window[14:0]
    wreg(fcal_pd_cal_addr, 0) # PD_CAL_FCAL
    wreg(fcal_start_addr, 0) # fcal_start
    vco_min = 0
    vco_max = 0
    no_cross = True
    
    for vco in range(0, 128, 2):
        wreg(vcocap_addr, vco)   #    cap code  
        wreg(fcal_start_addr, 1)     # fcal_start
        count=0
        while rreg(fcal_done_addr)==0 and count<1000: count+=1  # fcal_done
        readout = rreg(fcal_cnt_op_addr) # fcal_cnt_op[15:0]
        wreg(fcal_start_addr, 0) # # fcal_start =0
        if (count>=1000):
            print "cal_done timeout at %d", vco
            continue
        else:
            if readout > window :
                vco_min = vco
            elif no_cross :
                vco_max = vco
                no_cross = False          
            #print "scan, %3d, %04x, %04x" % (vco, window, readout)
    vco = int((vco_min+vco_max)/2)
    wreg(vcocap_addr, vco)
    wreg(fcal_lo_open_addr, 0)
    wreg(fcal_window_addr, 0)
    wreg(fcal_pd_cal_addr, 1)
    #print "Top PLL %04X final vco = %3d" % (base,vco)    
    return vco

####################################################################################################
# set_lane_pll ()
#
# Programs TXPLL or RXPLL according to PLL parameter(s) passed
#
# returns status of PLL programming (success/fail)
#
#################################################################################################### 
def set_lane_pll (tgt_pll=None, datarate=None, fvco=None, cap=None, n=None, div4=None, div2=None, refclk=None, frac_en=None, frac_n=None, lane=None):

    if tgt_pll==None:
        if datarate==None:
            print" \tEnter one or more of the following arguments:"
            print" \tset_lane_pll(tgt_pll='both'/'rx'/'tx', datarate, fvco, cap, n, div4, div2, refclk, lane)"
            return
        else:
            tgt_pll='both' # set both PLLs with datarate passed
    
    fvco_max = 33.0
    fvco_min = 15.0
    pll_n_max = 511 # (0x1FF)

    lanes = get_lane_list(lane)      # determine lanes to work on
    #get_lane_mode(lanes)         # Get current PAM4/NRZ modes settings of the specified lane(s)
    pll_params = get_lane_pll(lanes) # Get current PLL settings of the specified lane(s)

    #pll_params = [list(x) for x in pll_params_curr]
    if 'tx' in tgt_pll:
        desired_pll = [0]
    elif 'rx' in tgt_pll:
        desired_pll = [1]
    else: #if tgt_pll=='both':
        desired_pll = [0,1]
    
    # Temporary holders for each lane/pll
    lane_datarate=[0,1]
    lane_fvco    =[0,1]
    lane_cap     =[0,1]
    lane_n       =[0,1]
    lane_div4    =[0,1]
    lane_div2    =[0,1]
    lane_refclk  =[0,1]
    lane_frac_n  =[0,1]
    lane_frac_en =[0,1]
  
    ####### determine desired PLL parameters if passed, otherwise re-use current values from chip
    for ln in lanes:
        get_lane_mode(ln)
        #### Do this per PLL (TXPLL and/or RXPLL) of each lane
        for pll in desired_pll:
            lane_cap   [pll] = pll_params[ln][pll][2] if    cap == None else     cap
            lane_n     [pll] = pll_params[ln][pll][3] if      n == None else       n
            lane_div4  [pll] = pll_params[ln][pll][4] if   div4 == None else    div4
            lane_div2  [pll] = pll_params[ln][pll][5] if   div2 == None else    div2
            lane_refclk[pll] = pll_params[ln][pll][6] if refclk == None else  refclk
            lane_frac_n[pll] = pll_params[ln][pll][7] if frac_n == None else  frac_n
            lane_frac_en[pll]= pll_params[ln][pll][8] if frac_en== None else  frac_en

            ####### Calculate 'desired_n' per lane per pll (only if Data Rate or Fvco is passed)
            if datarate != None or fvco != None:
                if datarate != None:
                    desired_data_rate = float(datarate)
                    if desired_data_rate > fvco_max:
                        data_rate_to_fvco_ratio = 2.0    ##### PAM4 Data Rate, > 33 Gbps
                        wreg(c.rx_pam4_en_addr,    1,ln) # RX_PAM4_EN=1
                        wreg(c.tx_nrz_mode_addr,   0,ln) # TX_NRZ_EN=0
                        wreg(c.tx_mode10g_en_addr, 0,ln) # TX_NRZ_10G_EN=0 (or Disable NRZ Half-Rate Mode)
                        wreg(c.rx_mode10g_addr,    0,ln) # RX_NRZ_10G_EN=0 (or Disable NRZ Half-Rate Mode)
                    elif desired_data_rate < fvco_min:
                        data_rate_to_fvco_ratio = 0.5    ##### NRZ Half-Rate Data Rate, < 15 Gbps
                        wreg(c.rx_pam4_en_addr,    0,ln) # RX_PAM4_EN=0
                        wreg(c.tx_nrz_mode_addr,   1,ln) # TX_NRZ_EN=1
                        wreg(c.tx_mode10g_en_addr, 1,ln) # TX_NRZ_10G_EN=1 (or Enable NRZ Half-Rate Mode)
                        wreg(c.rx_mode10g_addr,    1,ln) # RX_NRZ_10G_EN=1 (or Enable NRZ Half-Rate Mode)
                    else: 
                        data_rate_to_fvco_ratio = 1.0    ##### NRZ Full-Rate Data Rate, 15 Gbps to 33 Gbps
                        wreg(c.rx_pam4_en_addr,    0,ln) # RX_PAM4_EN=0
                        wreg(c.tx_nrz_mode_addr,   1,ln) # TX_NRZ_EN=1
                        wreg(c.tx_mode10g_en_addr, 0,ln) # TX_NRZ_10G_EN=0 (or Disable NRZ Half-Rate Mode)
                        wreg(c.rx_mode10g_addr,    0,ln) # RX_NRZ_10G_EN=0 (or Disable NRZ Half-Rate Mode)
                        
                    desired_fvco = desired_data_rate / data_rate_to_fvco_ratio
                else:
                    desired_fvco = float(fvco)
                    
                if desired_fvco >fvco_max: desired_fvco=fvco_max
                if desired_fvco <fvco_min: desired_fvco=fvco_min
                
                ### Now, calculate PLL_N per lane, from desired_fvco
                div_by_4 = 1.0 if lane_div4[pll]==0 else 4.0
                mul_by_2 = 1.0 if lane_div2[pll]==1 else 2.0
                
                desired_n = int( (1000.0 * desired_fvco * div_by_4 ) / ( 2.0 * mul_by_2 * lane_refclk[pll] ) )
                if desired_n > pll_n_max: desired_n = pll_n_max
                lane_n[pll]=desired_n

        #update this lane's TXPLL and RXPLL params
        pll_params[ln] = [(0,0,lane_cap[0],lane_n[0],lane_div4[0],lane_div2[0],lane_refclk[0],lane_frac_n[0],lane_frac_en[0]),(0,0,lane_cap[1],lane_n[1],lane_div4[1],lane_div2[1],lane_refclk[1],lane_frac_n[1],lane_frac_en[1])]
            
    ###### Now program both PLLs of the specified lane(s)
    for ln in lanes:
        wreg(c.rx_pll_pu_addr, 0, ln) # Power down RX PLL while prgramming PLL
        wreg(c.tx_pll_pu_addr, 0, ln) # Power down TX PLL while prgramming PLL
        
        if 'tx' in tgt_pll or tgt_pll=='both':
            wreg(c.tx_pll_lvcocap_addr, pll_params[ln][0][2], ln)
            wreg(c.tx_pll_n_addr,       int(pll_params[ln][0][3]), ln)
            wreg(c.tx_pll_div4_addr,    pll_params[ln][0][4], ln)
            wreg(c.tx_pll_div2_addr,    pll_params[ln][0][5], ln)
            wreg(c.tx_pll_frac_n_addr,  pll_params[ln][0][7], ln)
            wreg(c.tx_pll_frac_en_addr, pll_params[ln][0][8], ln)
    
        if 'rx' in tgt_pll or tgt_pll=='both':
            wreg(c.rx_pll_lvcocap_addr, pll_params[ln][1][2], ln)
            wreg(c.rx_pll_n_addr,       int(pll_params[ln][1][3]), ln)
            wreg(c.rx_pll_div4_addr,    pll_params[ln][1][4], ln)
            wreg(c.rx_pll_div2_addr,    pll_params[ln][1][5], ln)
            wreg(c.rx_pll_frac_n_addr,  pll_params[ln][1][7], ln)
            wreg(c.rx_pll_frac_en_addr, pll_params[ln][1][8], ln)
            
        wreg(c.rx_pll_pu_addr, 1, ln) # Power up RX PLL after prgramming PLL
        wreg(c.tx_pll_pu_addr, 1, ln) # Power up TX PLL after prgramming PLL

    
####################################################################################################
# get_lane_pll ()
#
# reads all PLL related registers and computes PLL Frequencies and Data Rates
#
# returns all TXPLL and RXPLL parameters for the specified lane(s)
#
####################################################################################################    
def get_lane_pll (lane=None):

    lanes = get_lane_list(lane)
    #get_lane_mode(lanes)

    pll_params = {}
    
    #ref_clk=195.3125
    #ref_clk=156.25
    ref_clk = gRefClkFreq
    

    for ln in lanes:
        tx_div4_en        = rreg(c.tx_pll_div4_addr,       ln)
        tx_div2_bypass    = rreg(c.tx_pll_div2_addr,       ln)
        tx_pll_n          = rreg(c.tx_pll_n_addr,          ln)
        tx_pll_cap        = rreg(c.tx_pll_lvcocap_addr,    ln)
        tx_10g_mode_en    = rreg(c.tx_mode10g_en_addr,     ln) # NRZ 10G (or NRZ Half-Rate Mode)
        tx_pll_frac_n     = rreg(c.tx_pll_frac_n_addr,     ln) # Fractional_PLL_N
        if chip_rev()==2.0: # For R2.0, TX Frac-N is 20 bits
            tx_pll_frac_n_hi = rreg([0x0d9,[3,0]],         ln) # Fractional_PLL_N[19:16]
            tx_pll_frac_n_lo = rreg(c.tx_pll_frac_n_addr,  ln) # Fractional_PLL_N[15:0]
            tx_pll_frac_n = (tx_pll_frac_n_hi << 16) + tx_pll_frac_n_lo
        tx_pll_frac_order = rreg(c.tx_pll_frac_order_addr, ln) # Fractional_PLL_ORDER
        tx_pll_frac_en    = rreg(c.tx_pll_frac_en_addr,    ln) # Fractional_PLL_EN
        
        rx_div4_en        = rreg(c.rx_pll_div4_addr,       ln)
        rx_div2_bypass    = rreg(c.rx_pll_div2_addr,       ln)
        rx_pll_n          = rreg(c.rx_pll_n_addr,          ln)
        rx_pll_cap        = rreg(c.rx_pll_lvcocap_addr,    ln)
       #rx_10g_mode_en    = rreg(c.rx_mode10g_addr,        ln) # checking TX 10G is enough
        rx_pll_frac_n     = rreg(c.rx_pll_frac_n_addr,     ln) # Fractional_PLL_N
        rx_pll_frac_order = rreg(c.rx_pll_frac_order_addr, ln) # Fractional_PLL_ORDER
        rx_pll_frac_en    = rreg(c.rx_pll_frac_en_addr,    ln) # Fractional_PLL_EN
        
        tx_div_by_4 = 1.0 if tx_div4_en==0     else 4.0
        rx_div_by_4 = 1.0 if rx_div4_en==0     else 4.0
        tx_mul_by_2 = 1.0 if tx_div2_bypass==1 else 2.0
        rx_mul_by_2 = 1.0 if rx_div2_bypass==1 else 2.0
        
        pam4_mode_en = 1 if (rreg(c.tx_nrz_mode_addr,ln) == 0 and rreg(c.rx_pam4_en_addr,ln) == 1) else 0
        if pam4_mode_en: ########## Lane is in PAM4 mode (i.e. 33G =< data rate < 63G)
            data_rate_to_fvco_ratio = 2.0 
        else: ##################### Lane is in NRZ mode
            if tx_10g_mode_en==0: # Lane is in NRZ Full Rate mode (i.e. 15G < data rate < 33G)
                data_rate_to_fvco_ratio = 1.0
            else:                 # Lane is in NRZ Half Rate mode (i.e. data rate =< 15G) 
                data_rate_to_fvco_ratio = 0.5
            
        if chip_rev() == 2.0:
            tx_pll_n_float = float(tx_pll_n) + float(tx_pll_frac_n/1048575.0) if tx_pll_frac_en else float(tx_pll_n)
        else:
            tx_pll_n_float = float(tx_pll_n) + float(tx_pll_frac_n/65535.0)  if tx_pll_frac_en else float(tx_pll_n)        

        rx_pll_n_float = float(rx_pll_n) + float(rx_pll_frac_n/65535.0) if rx_pll_frac_en else float(rx_pll_n)
        
        tx_fvco = (ref_clk * tx_pll_n_float * 2.0 * tx_mul_by_2) / tx_div_by_4 / 1000.0  # in GHz
        rx_fvco = (ref_clk * rx_pll_n_float * 2.0 * rx_mul_by_2) / rx_div_by_4 / 1000.0  # in GHz

        tx_data_rate = tx_fvco * data_rate_to_fvco_ratio 
        rx_data_rate = rx_fvco * data_rate_to_fvco_ratio 
        
        tx_pll_params = tx_data_rate, tx_fvco, tx_pll_cap, tx_pll_n_float, tx_div4_en, tx_div2_bypass, ref_clk, tx_pll_frac_en, tx_pll_frac_n
        rx_pll_params = rx_data_rate, rx_fvco, rx_pll_cap, rx_pll_n_float, rx_div4_en, rx_div2_bypass, ref_clk, rx_pll_frac_en, rx_pll_frac_n
        
        pll_params[ln] = [tx_pll_params,rx_pll_params]
        
    return pll_params    #data_rate, fvco, pll_cap, pll_n, div4, div2, ref_clk
#################################################################################################### 
# pll()
#
# This is intended for python command line usage, to get or set PLL
# 
# See get_lane_pll() and  set_lane_pll() for actual PLL functions
#
####################################################################################################
def pll (tgt_pll=None, datarate=None, fvco=None, cap=None, n=None, div4=None, div2=None, refclk=None, frac_en=None, frac_n=None, lane=None):

    lanes = get_lane_list(lane)
    
    if datarate!=None or fvco!=None or cap!=None or n!=None or div4!=None or div2!=None or refclk!=None or frac_en!=None or frac_n!=None: # desired_pll_n or desired_data_rate, then program PLL registers
        set_lane_pll(tgt_pll, datarate, fvco, cap, n, div4, div2, refclk, frac_en, frac_n, lane)
    
    #get_lane_mode(lanes) # Now update the lanes' modes (PAM4 or NRZ) before printing them out
    pll_params = get_lane_pll(lanes)
    
    ##### Print Headers
    print("\n PLL Parameters for Slice %d with RefClk: %8.4f MHz\n"%(gSlice,pll_params[lanes[0]][0][6])),
    print("\n+-----------+------------- T X  P L L --------------------+------------- R X  P L L --------------------+"),
    print("\n|Lane| mode | DataRate,     Fvco, CAP,       N, DIV4, DIV2| DataRate,     Fvco, CAP,       N, DIV4, DIV2|"),
    print("\n+-----------+---------------------------------------------+---------------------------------------------+"),
    
    for ln in lanes:
        get_lane_mode(ln)
        print("\n|%3s | %4s |"%(lane_name_list[ln],gEncodingMode[gSlice][ln][0].upper())),
        for pll in [0,1]:
            print("%8.5f, %8.5f, %3d, %7.3f, %3d , %3d |" %(pll_params[ln][pll][0], pll_params[ln][pll][1], pll_params[ln][pll][2], pll_params[ln][pll][3],  pll_params[ln][pll][4],  pll_params[ln][pll][5])),
        if ln==lanes[-1] or ln==7:
            print("\n+-----------+---------------------------------------------+---------------------------------------------+"),
####################################################################################################
## EYE Margin, by Python directly accessing the HW registers
####################################################################################################
def eye(lane = None):
    lanes = get_lane_list(lane)
    result = {}
    fw_eye_en = fw_loaded(print_en=0) and fw_date(print_en=0)>=18015 and fw_reg_rd(128)!=0 # FW Date 20190429 and later
    for ln in lanes:    
        get_lane_mode(ln)
        line_encoding = lane_mode_list[ln].lower()
        c = Pam4Reg if line_encoding == 'pam4' else NrzReg
        x = 100 if line_encoding == 'pam4' else 200
        rdy = sum(ready(ln)[ln])        
        em=[-1,-1,-1]
        ##### PAM4 EYE
        if line_encoding == 'pam4' and rdy == 3:
            ####### FW-based Eye margin
            # if FW is loaded and the Background FW is not paused
            # if fw_loaded and fw_pause(print_en=0)[0][2] == 1:
            if fw_eye_en == True:        
                fw_debug_cmd(section=10, index=5, lane=ln)
                em = [rreg(0x9f00+eye_index) for eye_index in range(3)]
            ####### SW-based Eye margin. Direct access to HW registers
            else:
                eye_margin = []
                dac_val = dac(lane=ln)[ln]
                for eye_index in range (0,3):
                    result1 = 0xffff
                    for y in range (0,4):
                        sel = 3 * y + eye_index
                        wreg(c.rx_plus_margin_sel_addr, sel, ln)
                        plus_margin = rreg(c.rx_plus_margin_addr, ln)
                        if (plus_margin > 0x7ff):
                            plus_margin = plus_margin - 0x1000
                        wreg(c.rx_minus_margin_sel_addr, sel, ln)
                        minus_margin = (rreg(c.rx_minus_margin_addr, ln))
                        if (minus_margin > 0x7ff):
                            minus_margin = minus_margin - 0x1000
                        diff = plus_margin - minus_margin
                        if ( diff < result1):
                            result1 = diff
                        else:
                            result1 = result1
                    eye_margin.append((result1))
                    em[eye_index] = (float(eye_margin[eye_index])/2048.0) * (x + (50.0 * float(dac_val)))
        ##### NRZ EYE
        elif line_encoding == 'nrz' and rdy == 3: 
            dac_val = dac(lane=ln)[ln]
            eye_reg_val = rreg(c.rx_em_addr, ln)
            em[0] = (float(eye_reg_val) / 2048.0) * (x + (50.0 * float(dac_val)))            
            em[1]=0;em[2]=0
 
        result[ln] = int(em[0]),int(em[1]),int(em[2])        
    return result
####################################################################################################
## EYE Margin, by Python directly accessing the HW registers
####################################################################################################
def sw_eye(lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for ln in lanes:    
        get_lane_mode(ln)
        line_encoding = lane_mode_list[ln].lower()
        c = Pam4Reg if line_encoding == 'pam4' else NrzReg
        x = 100 if line_encoding == 'pam4' else 200
        rdy = sum(ready(ln)[ln])
        em=[-1,-1,-1]
        ##### PAM4 EYE
        if line_encoding == 'pam4' and rdy == 3:
            eye_margin = []
            ####### FW-based Eye margin
            # if FW is loaded and the Background FW is not paused
            # if fw_loaded and fw_pause(print_en=0)[0][2] == 1:
            if False:# fw_loaded and fw_reg_rd(128)!=0:
                print ("FW EYE MARGIN Lane %d"%ln)
                fw_debug_cmd(section=10, index=5, lane=ln)
                em = [rreg(0x9f00+eye_index) for eye_index in range(3)]
            ####### SW-based Eye margin. Direct access to HW registers
            else:
                dac_val = dac(lane=ln)[ln]
                for eye_index in range (0,3):
                    result1 = 0xffff
                    for y in range (0,4):
                        sel = 3 * y + eye_index
                        wreg(c.rx_plus_margin_sel_addr, sel, ln)
                        plus_margin = rreg(c.rx_plus_margin_addr, ln)
                        if (plus_margin > 0x7ff):
                            plus_margin = plus_margin - 0x1000
                        wreg(c.rx_minus_margin_sel_addr, sel, ln)
                        minus_margin = (rreg(c.rx_minus_margin_addr, ln))
                        if (minus_margin > 0x7ff):
                            minus_margin = minus_margin - 0x1000
                        diff = plus_margin - minus_margin
                        if ( diff < result1):
                            result1 = diff
                        else:
                            result1 = result1
                    eye_margin.append((result1))
                    em[eye_index] = (float(eye_margin[eye_index])/2048.0) * (x + (50.0 * float(dac_val)))
        ##### NRZ EYE
        elif line_encoding == 'nrz' and rdy == 3: 
            dac_val = dac(lane=ln)[ln]
            eye_reg_val = rreg(c.rx_em_addr, ln)
            em[0] = (float(eye_reg_val) / 2048.0) * (x + (50.0 * float(dac_val)))            
            em[1]=0;em[2]=0
 
        result[ln] = int(em[0]),int(em[1]),int(em[2])        
    return result
####################################################################################################
## EYE Margin, by Firmware
####################################################################################################
def fw_eye(lane = None):

    fw_eye_en = fw_loaded(print_en=0) and fw_date(print_en=0)>=18015 and fw_reg_rd(128)!=0 # FW Date 20190429 or later
    if fw_eye_en == False:
        print("\n*** FW Not Loaded, or "),
        print("\n*** FW Eye Margin function Not Available In This Release (Need DateCode 20190429 or later), or"),
        print("\n*** FW Background Functions are Disabled (FW REG 128 = 0 instead of 0xFFFF)\n"),
        return -1, -1, -1
        
    lanes = get_lane_list(lane)
    result = {}
    for ln in lanes:    
        get_lane_mode(ln)
        line_encoding = lane_mode_list[ln].lower()
        c = Pam4Reg if line_encoding == 'pam4' else NrzReg
        x = 100 if line_encoding == 'pam4' else 200
        rdy = sum(ready(ln)[ln])
        em=[-1,-1,-1]
        ##### PAM4 EYE
        if line_encoding == 'pam4' and rdy == 3:
            fw_debug_cmd(section=10, index=5, lane=ln)
            em = [rreg(0x9f00+eye_index) for eye_index in range(3)]
        ##### NRZ EYE
        elif line_encoding == 'nrz' and rdy == 3: # NRZ EYE
            dac_val = dac(lane=ln)[ln]
            eye_reg_val = rreg(c.rx_em_addr, ln)
            em[0] = (float(eye_reg_val) / 2048.0) * (x + (50.0 * float(dac_val)))            
            em[1] = 0
            em[2] = 0
        result[ln] = int(em[0]),int(em[1]),int(em[2])                
    return result
def fw_eye_test (lane = None):
    lanes = get_lane_list(lane)
    for ln in lanes:
        fw_debug_cmd(section=10, index=5, lane=ln)
        em = [rreg(0x9f00+eye_index) for eye_index in range(3)]
        print em

####################################################################################################
## PAM4 EYE Margin, by FW or by Python directly accessing HW registers
####################################################################################################
def eye_pam4(lane=None):
    c = Pam4Reg
    #c.rx_plus_margin_sel_addr  = [0x88,[15,12]]
    #c.rx_minus_margin_sel_addr = [0x88,[11,8]]
    #c.rx_plus_margin_addr = [0x32,[15,4]]
    #c.rx_minus_margin_addr = [0x32, [3,0], 0x33, [15,8]]
    #c.rx_minus_margin_upper_addr = [0x32,[3,0]]
    #c.rx_minus_margin_lower_addr = [0x33,[15,8]]

    fw_eye_en = fw_loaded(print_en=0) and fw_date(print_en=0)>=18015 and fw_reg_rd(128)!=0 # FW Date 20190429 and later
    
    lanes = get_lane_list(lane)
    eyes = {}
    for ln in lanes:     
        get_lane_mode(ln)
        line_encoding = lane_mode_list[ln].lower()
        c = Pam4Reg if line_encoding == 'pam4' else NrzReg
        x = 100 if line_encoding == 'pam4' else 200
        rdy = sum(ready(ln)[ln])
        em=[-1,-1,-1]
        if line_encoding == 'pam4' and rdy == 3:
            ####### FW-based PAM4 Eye margin
            # if FW is loaded and the Background FW is not paused
            # if fw_loaded and fw_pause(print_en=0)[0][2] == 1:
            if fw_eye_en == True:      
                fw_debug_cmd(section=10, index=5, lane=ln)
                em = [rreg(0x9f00+eye_index) for eye_index in range(3)]
                eyes[ln] = em[0], em[1], em[2]
            ####### SW-based PAM4 Eye margin. Direct access to HW registers
            else:
                eye_margin = []        
                dac_val = dac(lane=ln)[ln]
                for eye_index in range (3):
                    result1 = 0xffff
                    for y in range (4):
                        sel = 3 * y + eye_index
                        wreg(c.rx_minus_margin_sel_addr, sel, ln)
                        wreg(c.rx_plus_margin_sel_addr, sel, ln)
                        plus_margin = rreg(c.rx_plus_margin_addr, ln)
                        if (plus_margin > 0x7ff):
                            plus_margin = plus_margin - 0x1000
                        minus_margin = (rreg(c.rx_minus_margin_addr, ln))
                        if (minus_margin > 0x7ff):
                            minus_margin = minus_margin - 0x1000
                        diff = plus_margin - minus_margin
                        if ( diff < result1):
                            result1 = diff
                        else:
                            result1 = result1
                        #result1 = result1 + diff
                    eye_margin.append((result1))
                em0 = (float(eye_margin[0])/2048.0) * (x + (50.0 * float(dac_val)))
                em1 = (float(eye_margin[1])/2048.0) * (x + (50.0 * float(dac_val)))
                em2 = (float(eye_margin[2])/2048.0) * (x + (50.0 * float(dac_val)))
                eyes[ln] = em0, em1, em2
    return eyes
 ####################################################################################################
# Resets the PRBS Error Counter for the selected lane
# 
####################################################################################################
def prbs_rst(lane = None):
    lanes = get_lane_list(lane)
    for lane in lanes:
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg
        wreg(c.rx_err_cntr_rst_addr, 0, lane)
        time.sleep(0.001)
        wreg(c.rx_err_cntr_rst_addr, 1, lane)
        time.sleep(0.001)
        wreg(c.rx_err_cntr_rst_addr, 0, lane)
        
        
####################################################################################################
# Collects the PRBS Error Counter for the selected lane
# 
# It resets the counter first if rst=1
####################################################################################################
def get_prbs(lane = None, rst=0, print_en=1):
    lanes = get_lane_list(lane)
    
    ###### 1. Clear Rx PRBS Counter for these lanes, if requested
    if (rst==1): 
        prbs_rst(lane=lanes)
        
    result = {}
    ###### 2. Capture PRBS Counter for this lane
    for ln in lanes:
        c = Pam4Reg if lane_mode_list[ln].lower() == 'pam4' else NrzReg
        result[ln] = long(rreg(c.rx_err_cntr_msb_addr, ln)<<16) + rreg(c.rx_err_cntr_lsb_addr, ln)
 
    return result
####################################################################################################
# Collects the PRBS Error Counter for the selected lane
# 
# It resets the counter first if rst=1
#################################################################################################### 
def get_prbs_cnt (lane=0): 
    global gPrbsPrevCount
    global gPrbsPrevCountHi
    lane_status = LANE_RDY
    if(rregBits(sd_rdy_addr, rdy_bit_loc, lane) ==0): # check for PHY_RDY first
        cnt=8000000000L # return artifically large number
        gPrbsPrevCount[lane] =0
        gPrbsPrevCountHi[lane] = 0
        lane_status = LANE_NOT_RDY
    else:
        PrbsCountLo = rregLane(prbs_cnt_addr_lo,lane)
        PrbsCountHi = rregLane(prbs_cnt_addr_hi,lane)
        cnt = PrbsCountHi*65536+PrbsCountLo
        if(cnt < gPrbsPrevCount[lane]): # check PRBS counter wrapped around
            if (gPrbsPrevCountHi > PrbsCountHi):
                cnt = 9000000000L # return artifically large number
                gPrbsPrevCount[lane] =0
                gPrbsPrevCountHi[lane] = 0
                lane_status = LANE_NOT_GOOD
            else:
                PrbsCountLo = rregLane(prbs_cnt_addr_lo,lane)
                PrbsCountHi = rregLane(prbs_cnt_addr_hi,lane)
                cnt = PrbsCountHi*65536+PrbsCountLo
                gPrbsPrevCount[lane] = cnt # valid prbs count
                gPrbsPrevCountHi[lane] = PrbsCountHi
        else:
            gPrbsPrevCount[lane] = cnt # valid prbs count
            gPrbsPrevCountHi[lane] = PrbsCountHi
        
    return cnt, lane_status
####################################################################################################
# 
# flips the RX polarity, but user can ask to flip the TX polarity instead
#
####################################################################################################
def flip_pol(lane=None, port='rx', print_en=1):

    if lane==None and (type(lane)!=int or type(lane)!=list):
        print("\n   ...Usage.................................................................")
        print("\n      flip_pol (lane=0)            # flip RX polarity of lane 0"),
        print("\n      flip_pol (lane=0, port='rx') # flip RX polarity of lane 0"),
        print("\n      flip_pol (lane=0, port='TX') # flip TX polarity of lane 0"),
        print("\n      flip_pol (lane=[0,3,5])      # flip RX polarities of lanes 0, 3 and 5"),
        print("\n   .........................................................................")
        return

    lanes = get_lane_list(lane)
    result={}
    for ln in lanes:
        orig_tx_pol,orig_rx_pol =  pol(lane=ln, print_en=0)
        print("\nLane %d, TX Pol: (%d -> %d), RX Pol: (%d -> %d)"%(ln,orig_tx_pol,int(not orig_tx_pol),orig_rx_pol,int( not orig_rx_pol))),
        if port.upper() != 'TX': # if asked to reverse RX Polarity (default selection)
            new_tx_pol= orig_tx_pol
            new_rx_pol= int(not orig_rx_pol)
        else: # if asked to reverse TX Polarity
            new_tx_pol= int(not orig_tx_pol)
            new_rx_pol= orig_rx_pol
        
        pol(tx_pol=new_tx_pol, rx_pol= new_rx_pol, lane=ln, print_en=0)
            
        result[ln] = pol(lane=ln, print_en=0)
        #print result[ln]
    
    if print_en: 
        pol(lane=range(len(lane_name_list)), print_en=1)
        
    if print_en==0: return result
####################################################################################################
# 
# flips the RX or TX polarity
#
####################################################################################################
def flip_pol_all_combinations(lane=None, port='rx',print_en=1, function=None, *args):

  
    flip_list = range(0, 2^len(lane))
    lanes = get_lane_list(lane)
    result={}
    print flip_list
    pol()
    for flip in flip_list:
        i_ln=0
        for ln in lanes:
            if port.upper() != 'TX': # if asked to reverse RX Polarity (default selection)
                pol(tx_pol=None, rx_pol= flip_list[i_ln], lane=ln, print_en=0)
            else: # if asked to reverse TX Polarity
               pol(tx_pol=flip_list[i_ln], rx_pol=None , lane=ln, print_en=0)
            i_ln+=1
        
        
            
        #result[ln] = pol(lane=ln, print_en=0)
        pol()
    
    if print_en: 
        pol(lane=range(len(lane_name_list)), print_en=1)
        
    if print_en==0: return result
####################################################################################################
#
####################################################################################################
def scan_lane_partner(slice=[0,1], lane='all', print_en=True):

    global LanePartnerMap
    
    TX_lanes  = get_lane_list(lane)
    TX_slices = get_slice_list(slice)
    rx_slices=[0,1]
    rx_lanes=range(16)
    rx_partner=[-1,-1]

    ### Create List for LanePartnerMap
    # tx_map_slice_lane=[]
    # for slc in range(2):
        # tx_map_slice_lane.append([])
        # for ln in range(16):
            # tx_map_slice_lane[slc].append([])
    
    ### Turn all target lanes' TX output
    for TX_slc in TX_slices:
        sel_slice(TX_slc)
        tx_output(mode='on', lane=TX_lanes, print_en=0)

    if print_en:
        print("\n Scanning for Lane Partners...\n"),
        print("\n Slice_Lane               Slice_Lane"),
        print("\n         TX <-----------> RX "),

    ### Loop through all target slices/lanes (and find each one's RX partner)
    for TX_slc in TX_slices:
        for TX_ln in TX_lanes:
            if print_en:
                if TX_ln%4==0: print""
                print("\n    [S%d_%s]"%(TX_slc,lane_name_list[TX_ln])),
            num_lanes_checked=0
            lane_not_rdy_cnt=0
            ### Turn off this lane's TX
            sel_slice(TX_slc)
            tx_output(mode='off', lane=TX_ln, print_en=0)
            
            ### Search for RX partner of this TX
            for rx_slc in rx_slices:
                sel_slice(rx_slc)
                rdy_rx_ln = {}
                for rx_ln in rx_lanes:
                    rdy_rx_ln[rx_ln] = phy_rdy(rx_ln)[rx_ln]
                    #print("\n RX Slice %d Lane %d RDY = %d"%(rx_slc,rx_ln,rdy_rx_ln[rx_ln])),
                    if rdy_rx_ln[rx_ln]==0:# and lane_not_rdy_cnt==0:
                        rx_partner=[rx_slc,rx_ln]
                        conn = '<-LoopBack-->' if [rx_slc,rx_ln] == [TX_slc, TX_ln] else '<---Cable--->'
                        print "%s [S%d_%s]" % (conn, rx_slc,lane_name_list[rx_ln]),
                        LanePartnerMap[TX_slc][TX_ln] = [rx_slc,rx_ln]
                        lane_not_rdy_cnt+=1
                    # else:
                        # print "\nSlice %d Lane %d" % (rx_slc,rx_ln),                
                    num_lanes_checked+=1
                    
            ### Found this lane's partner. Turn its TX back on. Wait for its partner's RX to come ready before moving on to next TX_lane     
            sel_slice(TX_slc)
            tx_output(mode='on', lane=TX_ln, print_en=0)
            start=time.time()
            wait_time=time.time()-start
            sel_slice(rx_partner[0])
            while phy_rdy(rx_partner[1])[rx_partner[1]] == 0:
                wait_time=time.time()-start
                if wait_time>2.0:
                    break
            if print_en and lane_not_rdy_cnt!=1: 
                print(" (%d found for this lane, wait time = %2.1f)"%(lane_not_rdy_cnt, wait_time)),               
    if print_en==0:
        return LanePartnerMap
####################################################################################################
# Automatically determines the correct polarity of the RX input, based on PRBS Error Counter
# 
# By default flips the RX polarity if detects a wrong polarity, but user can ask to flip the TX polarity instead
#
####################################################################################################
def auto_pol(port='rx', tx_prbs='en', print_en=1):
    #print'here'
    lanes = range(0,16) #get_lane_list(lane)
    #get_lane_mode(lanes)
    
    result={}

    if print_en:
        tx_gen = 'TX PRBS Gen Forced Enabled' if 'en' in tx_prbs else 'TX PRBS Gen NOT Forced Enabled'
        print("\n\n...Slice %d Lanes %d-%d Auto Polarity with %s..."%(gSlice,lanes[0],lanes[-1], tx_gen)),
        print("\n                              #   Lane No: [A0.A1.A2.A3.A4.A5.A6.A7,B0.B1.B2.B3.B4.B5.B6.B7 ]"),
        if port.upper() != 'TX':
            print("\nRxPolarityMap.append([]); RxPolarityMap[%d]=["%(gSlice) ),
        else:
            print("\nTxPolarityMap.append([]); TxPolarityMap[%d]=["%(gSlice) ),
    
    # Make sure both PRBS TX Generator and RX Checker are enabled and both are set to PRBS31
    for ln in lanes:
        get_lane_mode(ln)
        c = Pam4Reg if lane_mode_list[ln].lower() == 'pam4' else NrzReg
        
        tx_prbs_gen_en = rreg([0x0a0,[14]],ln) # NRZ/PAM4 mode, PRBS Gen clock en
        rx_prbs_checker_en = rreg([0x043, [3]],ln) if c==Pam4Reg else rreg([0x161,[10]],ln) # PAM4 or NRZ mode, PRBS Sync Checker powered up
        if (tx_prbs =='en' and tx_prbs_gen_en==0): # or rx_prbs_checker_en==0:
            prbs_mode_select(lane=ln, prbs_mode='prbs')
    
    for ln in lanes:
        c = Pam4Reg if lane_mode_list[ln].lower() == 'pam4' else NrzReg
        
        if (tx_prbs =='en'): 
            prbs_mode_select(lane=ln, prbs_mode='prbs')
        
        ##Clear and read Rx PRBS Counter for this lane
        prbs_cnt_before = get_prbs(lane = ln, rst=1, print_en=0)[ln]
        
        orig_tx_pol,orig_rx_pol =  pol(tx_pol=None, rx_pol=None, lane=ln, print_en=0)

        if port.upper() != 'TX': # if asked to reverse RX Polarity (default selection)
            pol(tx_pol= orig_tx_pol, rx_pol=(int(not orig_rx_pol)), lane=ln, print_en=0)
        else: # if asked to reverse TX Polarity
            pol(tx_pol=(int(not orig_tx_pol)), rx_pol= orig_rx_pol, lane=ln, print_en=0)
        
        prbs_cnt_after = get_prbs(lane = ln, rst=1, print_en=0)[ln]
            
        if (prbs_cnt_before==0) or (prbs_cnt_before * 10 <= prbs_cnt_after): # if at least one order not improved, keep the orginal polarity
            pol(tx_pol= orig_tx_pol, rx_pol=orig_rx_pol, lane=ln, print_en=0)

        result[ln] = pol(tx_pol=None, rx_pol=None, lane=ln, print_en=0)
          
        #print prbs_cnt_before, prbs_cnt_after
        if print_en:
            if port.upper() != 'TX':
                if ln != lanes[-1]: print('%d,'%(result[ln][1])),
                else:               print('%d' %(result[ln][1])),
            else:
                if ln != lanes[-1]: print('%d,'%(result[ln][0])),
                else:               print('%d' %(result[ln][0])),
                
    #### Update the global polarity array for this Slice, os it can be used next time init() is called
    for ln in lanes:
        if port.upper() != 'TX':            
            RxPolarityMap[gSlice][ln]=result[ln][1] # gSlice lanes, RX Polarity
        else:
            TxPolarityMap[gSlice][ln]=result[ln][0] # gSlice lanes, TX Polarity
        
    if print_en:
        print("] # Slice %d lanes, %s Polarity"%(gSlice,port.upper()) )
    else:
        return result
    
####################################################################################################
# flips RX or TX polarities until fec_status is clean
# 
# call this only when set up in Gearbox mode
#
####################################################################################################
def auto_pol_fec(port='rx',lanes=range(0,16),print_en=1):
    #get_lane_list(lane)
    #get_lane_mode(lanes)
    
    result={}
    
    if print_en:
        print("\n                              #   Lane No: [A0.A1.A2.A3.A4.A5.A6.A7,B0.B1.B2.B3.B4.B5.B6.B7 ]"),
        if port.upper() != 'TX':
            print("\nRxPolarityMap.append([]); RxPolarityMap[%d]=["%(gSlice) ),
        else:
            print("\nTxPolarityMap.append([]); TxPolarityMap[%d]=["%(gSlice) ),
    
  
    for ln in lanes:
        get_lane_mode(ln)
        c = Pam4Reg if lane_mode_list[ln].lower() == 'pam4' else NrzReg
        if port.upper() != 'TX':
            ##read FEC Status and FIFO Counters for this gearbox
            overall_fec_stat, fec_statistics = fec_status(print_en=0)
            
            # if FEC Status is clean, done
            if not (-1 in overall_fec_stat):
                break
            
            fw_reg_wr(9,0x0000) # disable gearbox FW top control
            fw_reg_wr(8,0x0000) # disable FW PHY adaptation control
            #### 1 #### start with set all polarities to default pol(1,0)
            #### if all B-side adapt-done=1, flip RX pol until all FEC AMlock bits are '1'
            #### if all A-side adapt-done=1, flip RX pol until all FEC AMlock bits are '1'
            #### if any of A-side or B-side adapt-done=0, go to next and set FW-reg8=0xFFFF

            fw_reg_wr(9,0x0000) # disable gearbox FW top control
            fw_reg_wr(8,0xffff) # enable FW PHY adaptation control        
            #### 2 #### flip RX polarities until all lanes' RX adapt-done. If A-side adapt-done=0, wait long enough for it
            
            fw_reg_wr(9,0xffff) # enable back gearbox FW top control
            fw_reg_wr(8,0x0000) # disable FW PHY adaptation control
            #### 3 #### flip RX polarities until all remaining FEC AMlock bits are '1'
            
            fw_reg_wr(9,0xffff) # enable back gearbox FW top control
            fw_reg_wr(8,0xffff) # enable FW PHY adaptation control   
            #### 4 #### wait until all remaining FEC FIFO pointers fall into place
        #else:
        #### 5 #### flip TX polarities if partner FEC is not locked        
        
        orig_tx_pol,orig_rx_pol =  pol(tx_pol=None, rx_pol=None, lane=ln, print_en=0)

        if port.upper() != 'TX': # if asked to reverse RX Polarity (default selection)
            #pol(tx_pol= orig_tx_pol, rx_pol=~orig_rx_pol, lane=ln, print_en=0)
            flip_pol(lane=ln, port='rx', print_en=1)
        else: # if asked to reverse TX Polarity
            #pol(tx_pol=~orig_tx_pol, rx_pol= orig_rx_pol, lane=ln, print_en=0)
            flip_pol(lane=ln, port='tx', print_en=1)
        
        #prbs_cnt_after = get_prbs(lane = ln, rst=1, print_en=0)[ln]
            
        #if (prbs_cnt_before==0) or (prbs_cnt_before * 10 <= prbs_cnt_after): # if at least one order not improved, keep the orginal polarity
        #    pol(tx_pol= orig_tx_pol, rx_pol=orig_rx_pol, lane=ln, print_en=0)

        result[ln] = pol(tx_pol=None, rx_pol=None, lane=ln, print_en=0)
          
        #print prbs_cnt_before, prbs_cnt_after
        if print_en:
            if port.upper() != 'TX':
                if ln != lanes[-1]: print('%d,'%(result[ln][1])),
                else:               print('%d' %(result[ln][1])),
            else:
                if ln != lanes[-1]: print('%d,'%(result[ln][0])),
                else:               print('%d' %(result[ln][0])),
                
    #### Update the global polarity array for this Slice, os it can be used next time init() is called
    for ln in lanes:
        if port.upper() != 'TX':            
            RxPolarityMap[gSlice][ln]=result[ln][1] # gSlice lanes, RX Polarity
        else:
            TxPolarityMap[gSlice][ln]=result[ln][0] # gSlice lanes, TX Polarity
        
    if print_en:
        print("] # Slice %d lanes, %s Polarity"%(gSlice,port.upper()) )
    else:
        return result
    
####################################################################################################
def rx_prbs_mode(patt = None, lane = None):
    lanes = get_lane_list(lane)
    nrz_prbs_pat  = ['PRBS9', 'PRBS15', 'PRBS23', 'PRBS31'] 
    pam4_prbs_pat = ['PRBS9', 'PRBS13', 'PRBS15', 'PRBS31']
    result = {}
    for lane in lanes:
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg
        if patt == None:
            checker = rx_checker(lane=lane)[lane]
            patt_v = rreg(c.rx_prbs_mode_addr, lane)   
            if lane_mode_list[lane].lower() == 'pam4':
                pat_sel = pam4_prbs_pat[patt_v]
            else:
                pat_sel = nrz_prbs_pat[patt_v]
            result[lane] = checker, pat_sel
        elif type(patt) == int:
            rx_checker(1,lane)
            wreg(c.rx_prbs_mode_addr, patt, lane)
        elif type(patt) == str:
            val = pam4_prbs_pat.index(patt) if gPam4_En else nrz_prbs_pat.index(patt)
            rx_checker(1,lane)
            wreg(c.rx_prbs_mode_addr, val, lane)
    else:
        if result != {}: return result
def rx_checker(status = None, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg
        if status == None:
            result[lane] = rreg(c.rx_prbs_checker_pu_addr, lane)
        else:
            wreg(c.rx_prbs_checker_pu_addr, status, lane)
    else:
        if result != {}: return result
            
####################################################################################################
def rx(lane = None):
    lanes = get_lane_list(lane)
    for lane in lanes:
        checker, prbs = rx_prbs_mode(lane=lane)[lane]
        em = eye(lane=lane)[lane]
        dac_val = dac(lane=lane)[lane]
        em_taps = eye_pam4(lane=lane)[lane] if lane_mode_list[lane].lower() == 'pam4' else nrz_dfe(lane=lane)[lane]
        ctle_val = ctle(lane=lane)[lane]
        delta_val = delta(lane=lane)[lane]
        ctle_map_val = ctle_map(ctle_val,lane=lane)[lane]
        dc_gain_val = dc_gain(lane=lane)[lane]
        skef_en, skef_val = skef(lane=lane)[lane]
        edge_val = edge(lane=lane)[lane]
        print "%s, Rx checker: %s, %s, dac: %2d, eye: %6.2f, eye/taps: %s, ctle: %2d, %1d/%1d, skef: %s, %2d, AGC gain: %s, delta: %2d, edge: %s"%(lane_name_list[lane], bool(checker), prbs, dac_val, em, ','.join(map(str, em_taps)), ctle_val, ctle_map_val[0], ctle_map_val[1], skef_en, skef_val, ','.join(map(str, dc_gain_val)), delta_val, ','.join(map(str, edge_val)))
####################################################################################################
# Command to Disable (Squelch) or Enable (Unsquelch) TX output
#
# 'Disable' means put TX output in electrical idle mode
# 
####################################################################################################
def tx_output(mode=None,lane=None, print_en=1):

    hw_tx_control_addr  = 0xA0
    
    output_en_def  = ['ON','EN','UNSQ','UNSQUELCH']
    output_dis_def = ['OFF','DIS','SQ','SQUELCH']
    mode_def= ['EN','dis']

    lanes = get_lane_list(lane)
    result = {}
    
    if print_en: print("\n------------------------------"),
    if print_en: print("  Slice %d TX Output En or Dis Status Per Lane"%gSlice),
    if print_en: print("  ------------------------------"),
    if print_en: print("\n        Lane:"),
    if print_en: 
        for ln in range(16):
            print(" %4s" %(lane_name_list[ln])),

    if mode!=None: # Command to dis/en certain lanes' TX output
        for ln in lanes:
            if any(i in mode.upper() for i in output_en_def):   
                wreg([hw_tx_control_addr,[15]],1, ln) # stop using test pattern 0x0000
                wreg([hw_tx_control_addr,[14]],1, ln) # stop using test pattern 0x0000
                wreg([hw_tx_control_addr,[13]],1, ln) # stop using test pattern 0x0000
            if any(i in mode.upper() for i in output_dis_def):
                wreg(c.tx_test_patt4_addr, 0x0000, ln)
                wreg(c.tx_test_patt3_addr, 0x0000, ln)
                wreg(c.tx_test_patt2_addr, 0x0000, ln)
                wreg(c.tx_test_patt1_addr, 0x0000, ln)
                wreg([hw_tx_control_addr,[15]],0, ln) # use test pattern = 0x0000 to output electrical idle pattern
                wreg([hw_tx_control_addr,[14]],0, ln) # use test pattern = 0x0000 to output electrical idle pattern
                wreg([hw_tx_control_addr,[13]],1, ln) # use test pattern = 0x0000 to output electrical idle pattern


    for ln in range(16): # Read TX Output On/Off HW Register bit
        status2=rreg(hw_tx_control_addr,ln)
        status1=rreg([hw_tx_control_addr,[13]],ln)
        result[ln]=status1,status2
           
    if print_en: # Print Status
        print("\n TX Output  :"),
        for ln in range(16):
            print(" %4s"  %(mode_def[result[ln][0]])),
        print("\n TX Reg 0xA0:"),
        for ln in range(16):
            print(" %04X" %(result[ln][1])),
    else:
        return result
        
####################################################################################################
def tx_status(mode='func', lane = None): # mode= (1) 'off' or 'idle', (2) 'on' or 'func' or 'functional', (3) 'prbs'
    lanes = get_lane_list(lane)
    c = Pam4Reg
    for lane in lanes:
        #### OFF
        if mode.upper() == 'OFF' or mode.upper() == 'IDLE': # test pattern mode =0000       
            wreg(c.tx_test_patt4_addr, 0x0000, lane)
            wreg(c.tx_test_patt3_addr, 0x0000, lane)
            wreg(c.tx_test_patt2_addr, 0x0000, lane)
            wreg(c.tx_test_patt1_addr, 0x0000, lane)
            wreg(c.tx_test_patt_sc_addr,   0, lane) # 0xA0[15]=0, TX test signal source = Test pattern memory, not PRBS generator
            wreg(c.tx_test_patt_en_addr,   1, lane) # 0xA0[13]=1, TX data = test data, not traffic data
            wreg(c.tx_prbs_clk_en_addr,    0, lane) # 0xA0[14]=0, TX PAM4 PRBS Gen Clock disabled
            wreg(c.tx_prbs_en_addr,        0, lane) # 0xA0[11]=0, TX PAM4 PRBS Gen disabled 
            wreg(c.tx_prbs_clk_en_nrz_addr,0, lane) # 0xb0[14]=0, TX  NRZ PRBS gen clock disabled
            wreg(c.tx_prbs_en_nrz_addr,    0, lane) # 0xb0[11]=0, TX  NRZ PRBS gen disabled
        #### PRBS 
        elif 'PRBS' in mode.upper():      
            wreg(c.tx_test_patt_sc_addr,   1, lane) # 0xA0[15]=1, TX test signal source = Test pattern memory, not PRBS generator
            wreg(c.tx_test_patt_en_addr,   1, lane) # 0xA0[13]=1, TX data = test data, not functional/traffic mode
            wreg(c.tx_prbs_clk_en_addr,    1, lane) # 0xA0[14]=1, TX PAM4 PRBS GEN Clock enabled
            wreg(c.tx_prbs_en_addr,        1, lane) # 0xA0[11]=1, TX PAM4 PRBS Gen enabled 
            wreg(c.tx_prbs_clk_en_nrz_addr,1, lane) # 0xb0[14]=0, TX  NRZ PRBS gen clock disabled
            wreg(c.tx_prbs_en_nrz_addr,    1, lane) # 0xb0[11]=1, TX  NRZ PRBS gen enabled
        #### FUNCTIONAL (normal traffic mode)
        else:                    
            wreg(c.tx_test_patt_sc_addr,   0, lane) # 0xA0[15]=0, TX test signal source = not PRBS generator
            wreg(c.tx_test_patt_en_addr,   0, lane) # 0xA0[13]=0, TX data = functional mode, not test data
            wreg(c.tx_prbs_clk_en_addr,    0, lane) # 0xA0[14]=0, TX PAM4 PRBS Gen CLock disabled
            wreg(c.tx_prbs_en_addr,        0, lane) # 0xA0[11]=0, TX PAM4 PRBS Gen disabled 
            wreg(c.tx_prbs_clk_en_nrz_addr,0, lane) # 0xb0[14]=0, TX  NRZ PRBS gen clock disabled
            wreg(c.tx_prbs_en_nrz_addr,    0, lane) # 0xb0[11]=0, TX  NRZ PRBS gen disabled
                                     
####################################################################################################
def tx_test_patt(en=None, lane = None, tx_test_patt4_val=0x0000,tx_test_patt3_val=0x0000,tx_test_patt2_val=0x0000,tx_test_patt1_val=0x0000):
    lanes = get_lane_list(lane)
    c = Pam4Reg
    result = {}
    for lane in lanes:
        if en == None:
            prbs_en = rreg(c.tx_prbs_en_addr, lane)
            test_patt_en = rreg(c.tx_test_patt_en_addr, lane)
            prbs_clk_en = rreg(c.tx_prbs_clk_en_addr, lane)
            test_patt_sc = rreg(c.tx_test_patt_sc_addr, lane)
            val = prbs_en & test_patt_en & prbs_clk_en & (1-test_patt_sc)
            result[lane] = val
        elif en == 1:
            # test pattern mode        
            wreg(c.tx_test_patt_en_addr, en, lane)
            wreg(c.tx_test_patt_sc_addr, 1-en, lane)
            wreg(c.tx_test_patt4_addr, tx_test_patt4_val, lane)
            wreg(c.tx_test_patt3_addr, tx_test_patt3_val, lane)
            wreg(c.tx_test_patt2_addr, tx_test_patt2_val, lane)
            wreg(c.tx_test_patt1_addr, tx_test_patt1_val, lane)
        else:
            # normal traffic mode
            wreg(c.tx_test_patt_en_addr, 0x0, lane)
    else:
        if result != {}: return result
        
####################################################################################################
def tx_prbs_en(en = None, lane = None):
    lanes = get_lane_list(lane)
    c = Pam4Reg
    result = {}
    for lane in lanes:
        if en == None:
            prbs_en = rreg(c.tx_prbs_en_addr, lane)
            test_patt_en = rreg(c.tx_test_patt_en_addr, lane)
            prbs_clk_en = rreg(c.tx_prbs_clk_en_addr, lane)
            test_patt_sc = rreg(c.tx_test_patt_sc_addr, lane)
            val = prbs_en & test_patt_en & prbs_clk_en & test_patt_sc
            result[lane] = val
        else:
            wreg(c.tx_prbs_en_addr, en, lane)
            wreg(c.tx_test_patt_en_addr, en, lane)
            wreg(c.tx_prbs_clk_en_addr, en, lane)
            wreg(c.tx_test_patt_sc_addr, en, lane)
    
####################################################################################################
def tx_prbs_mode(patt = None, lane = None):
    lanes = get_lane_list(lane)
    c = Pam4Reg
    result = {}
    nrz_prbs_pat  = ['PRBS9', 'PRBS15', 'PRBS23', 'PRBS31'] 
    pam4_prbs_pat = ['PRBS9', 'PRBS13', 'PRBS15', 'PRBS31']
    for lane in lanes:
        if patt == None:
            prbs_en = rreg(c.tx_prbs_en_addr, lane)
            test_patt_en = rreg(c.tx_test_patt_en_addr, lane)
            prbs_clk_en = rreg(c.tx_prbs_clk_en_addr, lane)
            test_patt_sc = rreg(c.tx_test_patt_sc_addr, lane)
            patt_v = rreg(c.tx_prbs_patt_sel_addr, lane)
            if lane_mode_list[lane].lower() == 'pam4':
                pat_sel = pam4_prbs_pat[patt_v]
            else:
                pat_sel = nrz_prbs_pat[patt_v]
            result[lane] = (prbs_en & test_patt_en & prbs_clk_en & test_patt_sc, pat_sel)
        elif type(patt) == int:
            tx_prbs_en(0, lane)
            wreg(c.tx_prbs_patt_sel_addr, patt, lane)
            tx_prbs_en(1, lane)
        elif type(patt) == str:
            tx_prbs_en(0, lane)
            val = pam4_prbs_pat.index(patt) if lane_mode_list[lane].lower() == 'pam4' else nrz_prbs_pat.index(patt)
            wreg(c.tx_prbs_patt_sel_addr, val, lane)
            tx_prbs_en(1, lane)
    else:
        if result != {}: return result
        
####################################################################################################
def err_inject(lane = None):
    lanes = get_lane_list(lane)
    c = Pam4Reg
    for lane in lanes:
        wreg(c.tx_prbs_1b_err_addr, 0x0, lane)
        time.sleep(0.001)
        wreg(c.tx_prbs_1b_err_addr, 0x1, lane)
        time.sleep(0.001)
        wreg(c.tx_prbs_1b_err_addr, 0x0, lane)
    
####################################################################################################
def tx_taps(tap1 = None,tap2 = None,tap3 = None, tap4 = None, tap5 = None, tap6 = None, tap7 = None, tap8 = None, tap9 = None, tap10 = None, tap11 = None, lane = None):
    lanes = get_lane_list(lane)
    c = Pam4Reg
    result = {}
    for lane in lanes:
        if tap1==tap2==tap3==tap4==tap5==tap6==tap7==tap8==tap9==tap10==tap11==None:
            tap1_v = twos_to_int(rreg(c.tx_tap1_addr, lane), 8)
            tap2_v = twos_to_int(rreg(c.tx_tap2_addr, lane), 8)
            tap3_v = twos_to_int(rreg(c.tx_tap3_addr, lane), 8)
            tap4_v = twos_to_int(rreg(c.tx_tap4_addr, lane), 8)
            tap5_v = twos_to_int(rreg(c.tx_tap5_addr, lane), 8)
            tap6_v = twos_to_int(rreg(c.tx_tap6_addr, lane), 4)
            tap7_v = twos_to_int(rreg(c.tx_tap7_addr, lane), 4)
            tap8_v = twos_to_int(rreg(c.tx_tap8_addr, lane), 4)
            tap9_v = twos_to_int(rreg(c.tx_tap9_addr, lane), 4)
            tap10_v = twos_to_int(rreg(c.tx_tap10_addr, lane), 4)
            tap11_v = twos_to_int(rreg(c.tx_tap11_addr, lane), 4)
            result[lane] = tap1_v, tap2_v, tap3_v, tap4_v, tap5_v, tap6_v, tap7_v, tap8_v, tap9_v, tap10_v, tap11_v
        else:
            if tap1 != None: wreg(c.tx_tap1_addr, int_to_twos(tap1,8), lane)
            if tap2 != None: wreg(c.tx_tap2_addr, int_to_twos(tap2,8), lane)
            if tap3 != None: wreg(c.tx_tap3_addr, int_to_twos(tap3,8), lane)
            if tap4 != None: wreg(c.tx_tap4_addr, int_to_twos(tap4,8), lane)
            if tap5 != None: wreg(c.tx_tap5_addr, int_to_twos(tap5,8), lane)
            if tap6 != None: wreg(c.tx_tap6_addr, int_to_twos(tap6,4), lane)
            if tap7 != None: wreg(c.tx_tap7_addr, int_to_twos(tap7,4), lane)
            if tap8 != None: wreg(c.tx_tap8_addr, int_to_twos(tap8,4), lane)
            if tap9 != None: wreg(c.tx_tap9_addr, int_to_twos(tap9,4), lane)
            if tap10 != None: wreg(c.tx_tap10_addr, int_to_twos(tap10,4), lane)
            if tap11 != None: wreg(c.tx_tap11_addr, int_to_twos(tap11,4), lane)
    else:
        if result != {}: return result
    
####################################################################################################
def tx_taps_rule_check(lane = None):
    lanes = get_lane_list(lane)
    c = Pam4Reg
    result = {}
    for lane in lanes:
        taps = tx_taps(lane = lane)[lane]
        taps = [abs(taps[i]) for i in range(len(taps))]
        hf = rreg(c.tx_taps_hf_addr, lane)
        tx_taps_sum = sum(taps)/2.0 + sum([taps[i]*((hf>>i)&1) for i in range(5)])/2.0
        result[lane] = (tx_taps_sum <= c.tx_taps_sum_limit), tx_taps_sum
    else:
        if result != {}: return result
####################################################################################################
def tx(lane = None):
    lanes = get_lane_list(lane)
    for lane in lanes:
        test_patt_en = tx_test_patt(lane = lane)[lane]
        prbs_en, prbs_sel = tx_prbs_mode(lane = lane)[lane]
        taps = tx_taps(lane = lane)[lane]
        sum = tx_taps_rule_check(lane = lane)[lane][-1]
        print "%s, PRBS enable: %s %s, test pattern enable: %s, taps: %s, %d"%(lane_name_list[lane], bool(prbs_en), prbs_sel, bool(test_patt_en), ','.join((map(str,taps))), sum)

####################################################################################################
#
#   tx_sj( A = 6 ,f = 20, lane = 0 )
#
####################################################################################################
def tx_sj( A = 0.3, f = 2500, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    
    if (A!=None and A>0.0):
        sj_en = 1 
    elif (A!=None):
        sj_en = 0

    
    if A!=None:
        for ln in lanes:        
            if sj_en:
                tx_sj_en(en=1,lane=ln)
                tx_sj_ampl(A,ln)
                tx_sj_freq(f,ln)
            else: # Disable SJ
                tx_sj_en(en=0,lane=ln)
                
    for ln in lanes:       
        sj_en_status  = 'dis' if rregBits(0xf3,[5],ln)==1 else 'en'
        sj_amp =float(rregBits(0xf9,[15,13],ln))+ (float(rregBits(0xf9,[12,5],ln))/256.0)
        sj_freq=int(float(rregBits(0xf5,[15,2],ln)) * 3.2)
        result[ln]=sj_en_status,sj_amp,sj_freq
        
    return result
####################################################################################################
def tx_sj_range( A = 3, lane = 0):
    tx_sj_en(en=1,lane=lane)
    tx_sj_ampl(A,lane)
    wregBits(0xf5,[15,2],0x3fff,lane) # sj_freq = max
    
####################################################################################################
def tx_sj_en( en=1,lane=0):
    if en: 
        wregBits(0xf4,[0],0,lane)  # disable TRF before enabling injecting SJ 
        wregBits(0xf3,[5],0,lane)  # enable tx_rotator to inject SJ 
    else: # Disable SJ 
        wregBits(0xf3,[5],1,lane)  # disable tx_rotator
        tx_sj_ampl(A=0,lane=lane)  # sj_ampl_0123  = 0
        tx_sj_freq(f=0,lane=lane)  # sj_freq    = 0

    return en
####################################################################################################
def tx_sj_ampl( A = 3, lane = 0 ):
    addr = [0xf9,0xf8,0xf7,0xf6] # sj_ampl_0/1/2/3
    #           sj_ampl_cos_0,  sj_ampl_cos_1,         sj_ampl_cos_2,         sj_ampl_cos_3
    phase_value =[math.cos(0), math.cos((math.pi)/8), math.cos((math.pi)/4), math.cos(3*(math.pi)/8)]
    for i in range(4):
        value = A*phase_value[i]
        int_A = int(value)
        float_A = dec2bin(value - int_A)        
        wregBits(addr[i],[15,13], int_A,lane)
        wregBits(addr[i],[12,5],float_A,lane)

####################################################################################################
def tx_sj_freq( f = 250, lane = 0):
    data_f = int(round((f/3.2),0))   
    wregBits(0xf5,[15,2],data_f,lane) # sj_freq value
    

####################################################################################################
def lane_reset(lane = None):
    '''
    added super-cal disable/enable
    '''
    lanes = get_lane_list(lane)
    for lane in lanes:
        if lane_mode_list[lane].lower() == 'nrz':
            c = NrzReg
            wreg(c.rx_cntr_target_addr, 0x100, lane)
            time.sleep(0.02)
        else:
            c = Pam4Reg
        wreg(Pam4Reg.rx_mu_ow_addr, 0x0, lane)
        wreg(Pam4Reg.rx_mu_owen_addr, 0x1, lane)
        
        wreg(c.rx_lane_rst_addr, 0x1, lane)
        time.sleep(0.001)        
        wreg(c.rx_lane_rst_addr, 0x0, lane)
        wreg(Pam4Reg.rx_mu_ow_addr, 0x3, lane)
        wreg(Pam4Reg.rx_mu_owen_addr, 0x1, lane)
        if lane_mode_list[lane].lower() == 'nrz':
            rdy = wait_rdy(lane = lanes)
            wreg(c.rx_cntr_target_addr, 0x40, lane)
        
def wait_rdy(lane = None, t = 1):
    lanes = get_lane_list(lane)
    i = 0
    while(i<t):
        rdy = ready(lanes)
        tmp = [sum(rdy[lane]) == 2 for lane in lanes]
        em = [eye(lane)[lane][-1] for lane in lanes]
        if (False not in tmp) and (0.0 not in em): 
            break
        time.sleep(0.001)
        i += 0.001
    return rdy
    
def sm_cont(lane = None):
    lanes = get_lane_list(lane)
    for lane in lanes:
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
        wreg(c.rx_sm_cont_addr, 0x0, lane)
        time.sleep(0.001)
        wreg(c.rx_sm_cont_addr, 0x1, lane)
        #time.sleep(0.001)
        #wreg(c.rx_sm_cont_addr, 0x0, lane)
    
def bp1(en = None, st = 0, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
        if en == None:
            en_v = rreg(c.rx_bp1_en_addr, lane)
            st_v = rreg(c.rx_bp1_st_addr, lane)
            reached = rreg(c.rx_bp1_reached_addr, lane)
            result[lane] = en_v, st_v, reached
        else:
            wreg(c.rx_bp1_st_addr, st, lane)
            wreg(c.rx_bp1_en_addr, en, lane)
    else:
        if result != {}: return result
        
def bp2(en = None, st = 0, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
        if en == None:
            en_v = rreg(c.rx_bp2_en_addr, lane)
            st_v = rreg(c.rx_bp2_st_addr, lane)
            reached = rreg(c.rx_bp2_reached_addr, lane)
            result[lane] = en_v, st_v, reached
        else:
            wreg(c.rx_bp2_st_addr, st, lane)
            wreg(c.rx_bp2_en_addr, en, lane)
    else:
        if result != {}: return result   
        
def ctle(val = None, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
        if val == None:
            v = rreg(c.rx_agc_ow_addr, lane)
            result[lane] = v
        else:
            wreg(c.rx_agc_ow_addr, val, lane)
            wreg(c.rx_agc_owen_addr, 0x1, lane)
    else:
        if result != {}: return result
        
def ctle12 (ctle = None, ctle1 = None, ctle2 = None, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for ln in lanes:
        c = Pam4Reg if lane_mode_list[ln].lower() == 'pam4' else NrzReg  
        if ctle == None: # Read current CTLE settings
            ctle_idx = rreg(c.rx_agc_ow_addr, lane)
            ctle_1_bit4=rreg([0x1d7,[3]],ln)
            ctle_2_bit4=rreg([0x1d7,[2]],ln)
            ctle_1 = ctle_map(ctle_idx, lane=ln)[ln][0] + (ctle_1_bit4*8)
            ctle_2 = ctle_map(ctle_idx, lane=ln)[ln][1] + (ctle_2_bit4*8)            
            result[ln] = ctle_idx,ctle_1,ctle_2
        else:# Select requested CTLE index 
            wreg(c.rx_agc_ow_addr, ctle, ln)
            wreg(c.rx_agc_owen_addr, 0x1, ln)
            # overwrite its CTLE1/2 values if given
            if ctle1 != None and ctle2 != None: 
                if ctle1>7:
                    ctle_1_w = ctle1 - 8
                    wreg([0x1d7,[3]],1,ln)
                else:
                    ctle_1_w = ctle1
                    wreg([0x1d7,[3]],0,ln)
            
                if ctle2>7:
                    ctle_2_w = ctle2 - 8
                    wreg([0x1d7,[2]],1,ln)
                else:
                    ctle_2_w = ctle2
                    wreg([0x1d7,[2]],0,ln)       
                ctle_map(ctle, ctle1_w, ctle2_w, lane=ln)

    else:
        if result != {}: return result

def skef(en = None, val = None, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    c = Pam4Reg
    for lane in lanes:
        if en == None:
            en_v = rreg(c.rx_skef_en_addr, lane)
            v = rreg(c.rx_skef_degen_addr, lane)
            result[lane] = en_v, v
        else:
            wreg(c.rx_skef_degen_addr, val, lane)
            wreg(c.rx_skef_en_addr, en, lane)
    else:
        if result != {}: return result
        
def agcgain(agcgain1 = None, agcgain2 = None, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    c = Pam4Reg
    for lane in lanes:
        if agcgain1 == agcgain2 == None:
            agcgain1_v = Gray_Bin(rreg(c.rx_agcgain1_addr, lane))
            agcgain2_v = Gray_Bin(rreg(c.rx_agcgain2_addr, lane))
            result[lane] = agcgain1_v, agcgain2_v
        else:
            if agcgain1 != None:
                wreg(c.rx_agcgain1_addr, Bin_Gray(agcgain1), lane)
            if agcgain2 != None:
                wreg(c.rx_agcgain2_addr, Bin_Gray(agcgain2), lane)
    else:
        if result != {}: return result
        
def ffegain(ffegain1 = None, ffegain2 = None, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    c = Pam4Reg
    for lane in lanes:
        if ffegain1 == ffegain2 == None:
            ffegain1_v = Gray_Bin(rreg(c.rx_ffe_sf_msb_addr, lane))
            ffegain2_v = Gray_Bin(rreg(c.rx_ffe_sf_lsb_addr, lane))
            result[lane] = ffegain1_v, ffegain2_v
        else:
            if ffegain1 != None:
                wreg(c.rx_ffe_sf_msb_addr, Bin_Gray(ffegain1), lane)
            if ffegain2 != None:
                wreg(c.rx_ffe_sf_lsb_addr, Bin_Gray(ffegain2), lane)
    else:
        if result != {}: return result
         
def dc_gain(agcgain1=None, agcgain2=None, ffegain1=None, ffegain2=None, lane=None):    
    if agcgain1 == agcgain2 == ffegain1 == ffegain2 == None:
        agcgain_val = agcgain(lane=lane)
        ffegain_val = ffegain(lane=lane)
        result = {}
        for i in agcgain_val.keys():
            result[i] = agcgain_val[i][0], agcgain_val[i][1], ffegain_val[i][0], ffegain_val[i][1]
        else:
            if result != {}: return result
    else:
        if agcgain1 != None or agcgain2 != None:
            agcgain(agcgain1, agcgain2, lane)
        if ffegain1 != None or ffegain2 != None:
            ffegain(ffegain1, ffegain2, lane)
 
def ctle_map(sel = None, val1 = None, val2 = None, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
        if None in (sel, val1, val2):
            map0 = rreg(c.rx_agc_map0_addr, lane)
            map1 = rreg(c.rx_agc_map1_addr, lane)
            map2 = rreg(c.rx_agc_map2_addr, lane)
            agc = { 0: [map0>>13, (map0>>10) & 0x7],
                    1: [(map0>>7) & 0x7, (map0>>4) & 0x7],
                    2: [(map0>>1) & 0x7, ((map0 & 0x1)<<2) + (map1>>14)],
                    3: [(map1>>11) & 0x7, (map1>>8) & 0x7],
                    4: [(map1>>5) & 0x7, (map1>>2) & 0x7],
                    5: [((map1 & 0x3)<<1) + (map2>>15), (map2>>12) & 0x7],
                    6: [(map2>>9) & 0x7, (map2>>6) & 0x7],
                    7: [(map2>>3) & 0x7, map2 & 0x7]
                    }
            if sel == None:
                result[lane] = agc
            else:
                result[lane] = agc[sel]
        else:
            if sel != 2 and sel != 5:
                val = (val1<<3) + val2
                if sel == 0:
                    map0 = (rreg(c.rx_agc_map0_addr, lane) | 0xfc00) & (val<<10 | 0x3ff) 
                    wreg(c.rx_agc_map0_addr, map0, lane)
                elif sel == 1:
                    map0 = (rreg(c.rx_agc_map0_addr, lane) | 0x03f0) & (val<<4 | 0xfc0f) 
                    wreg(c.rx_agc_map0_addr, map0, lane)
                elif sel == 3:
                    map1 = (rreg(c.rx_agc_map1_addr, lane) | 0x3f00) & (val<<8 | 0xc0ff) 
                    wreg(c.rx_agc_map1_addr, map1, lane)
                elif sel == 4:
                    map1 = (rreg(c.rx_agc_map1_addr, lane) | 0x00fc) & (val<<2 | 0xff03) 
                    wreg(c.rx_agc_map1_addr, map1, lane)
                elif sel == 6:
                    map2 = (rreg(c.rx_agc_map2_addr, lane) | 0x0fc0) & (val<<6 | 0xf03f) 
                    wreg(c.rx_agc_map2_addr, map2, lane)
                elif sel == 7:
                    map2 = (rreg(c.rx_agc_map2_addr, lane) | 0x003f) & (val | 0xffc0) 
                    wreg(c.rx_agc_map2_addr, map2, lane)
            elif sel == 2:
                val = (val1<<1) + ((val2 & 0x4)>>2)
                map0 = (rreg(c.rx_agc_map0_addr, lane) | 0x000f) & (val | 0xfff0)
                wreg(c.rx_agc_map0_addr, map0, lane)
                val = val2 & 0x3
                map1 = (rreg(c.rx_agc_map1_addr, lane) | 0xc000) & ((val<<14) | 0x3fff)
                wreg(c.rx_agc_map1_addr, map1, lane)
            elif sel == 5:
                val = (val1 >>1 & 0x3)
                map1 = (rreg(c.rx_agc_map1_addr, lane) & 0xfffc) | (val & 0x0003)
                wreg(c.rx_agc_map1_addr, map1, lane)
                val = ((val1 & 0x1)<<3) + val2
                #map2 = (rreg(c.rx_agc_map2_addr, lane) | 0xf) & ((val<<12) | 0x0fff)
                map2 = (rreg(c.rx_agc_map2_addr, lane) & 0x0fff) | ((val<<12) & 0xf000)
                wreg(c.rx_agc_map2_addr, map2, lane)
    else:
        if result != {}:
            if sel == None:
                for key in range(8):
                    print key,
                    for lane in lanes:
                        print result[lane][key],
                    print ' '
            else:
                return result
 
                
def delta(val = None, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
        if val == None:
            result[lane] = twos_to_int(rreg(c.rx_delta_addr, lane), 7)
        else:
            wreg(c.rx_delta_ow_addr, int_to_twos(val, 7), lane)
            wreg(c.rx_delta_owen_addr, 0x1, lane)
    else:
        if result != {}: return result
        
def dac(val = None, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        get_lane_mode(lane)
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
        if val != None:
            wreg(c.rx_dac_ow_addr, val, lane)
            wreg(c.rx_dac_owen_addr, 1, lane)
        
        dac_v = rreg(c.rx_dac_addr, lane)
        result[lane] = dac_v
    else:
        if result != {}: return result        
def kp(val = None, lane = None, print_en=1):
    lanes = get_lane_list(lane)
    result = {}
    for ln in lanes:
        get_lane_mode(ln)
        c = Pam4Reg if lane_mode_list[ln].lower() == 'pam4' else NrzReg  
        if val != None:
            wreg(c.rx_kp_ow_addr, val, ln)
            wreg(c.rx_kp_owen_addr, 1, ln)
        
        ted_v = rreg(c.rx_ted_en_addr, ln)
        kp_v  = rreg(c.rx_kp_ow_addr, ln)
        result[ln] = ted_v, kp_v
        
    if print_en: # Print Status
        print("\nSlice %d, Lane:"%(sel_slice())),
        for ln in range(len(lane_name_list)):
            print(" %2s" %(lane_name_list[ln])),
        get_lane_mode('all')
        print("\n      CDR TED:"),
        for ln in range(len(lane_name_list)):
            c = Pam4Reg if lane_mode_list[ln] == 'pam4' else NrzReg
            ted_v = rreg(c.rx_ted_en_addr, ln)
            if lane_mode_list[ln] == 'pam4':
                print(" %2d"  %(ted_v)),
            else:
                print("  -"),
            
        print("\n      CDR Kp :"),
        for ln in range(len(lane_name_list)):
            c = Pam4Reg if lane_mode_list[ln] == 'pam4' else NrzReg
            kp_v  = rreg(c.rx_kp_ow_addr, ln)
            print(" %2d" %(kp_v)),
    else:
        if result != {}: return result        

def ted(val = None, lane = None, print_en=1):
    lanes = get_lane_list(lane)
    result = {}
    for ln in lanes:
        #get_lane_mode(ln)
        c = Pam4Reg  
        if val != None:
            wreg(c.rx_ted_en_addr, val, ln)
        
        ted_v = rreg(c.rx_ted_en_addr, ln)
        kp_v  = rreg(c.rx_kp_ow_addr, ln)
        result[ln] = ted_v, kp_v
        
    if print_en: # Print Status
        get_lane_mode('all')
        print("\nSlice %d, Lane:"%(sel_slice())),
        for ln in range(len(lane_name_list)):
            print(" %2s" %(lane_name_list[ln])),
        print("\n      CDR TED:"),
        for ln in range(len(lane_name_list)):
            c = Pam4Reg if lane_mode_list[ln] == 'pam4' else NrzReg
            ted_v = rreg(c.rx_ted_en_addr, ln)
            if lane_mode_list[ln] == 'pam4':
                print(" %2d"  %(ted_v)),
            else:
                print("  -"),
        print("\n      CDR Kp :"),
        for ln in range(len(lane_name_list)):
            c = Pam4Reg if lane_mode_list[ln] == 'pam4' else NrzReg
            kp_v  = rreg(c.rx_kp_ow_addr, ln)
            print(" %2d" %(kp_v)),
    else:
        if result != {}: return result        

def trf(val = None, lane = None, print_en=1):
    lanes = get_lane_list(lane)
    result = {}
    for ln in lanes:
        #get_lane_mode(ln)
        c = Pam4Reg  
        if val != None:
            wreg([0xf4,[0]], val, ln)
        
        trf_v = rreg([0xf4,[0]], ln)
        result[ln] = trf_v
        
    if print_en: # Print Status
        get_lane_mode('all')
        print("\nSlice %d, Lane:"%(sel_slice())),
        for ln in range(len(lane_name_list)):
            print(" %2s" %(lane_name_list[ln])),
        print("\n       TRF En:"),
        for ln in range(len(lane_name_list)):
            trf_v  = rreg([0xf4,[0]], ln)
            print(" %2d" %(trf_v)),
    else:
        if result != {}: return result        

def ready(lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        get_lane_mode(lane)
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
        sd = rreg(c.rx_sd_addr, lane)
        rdy = rreg(c.rx_rdy_addr, lane)
        adapt_done=1
        if fw_loaded(print_en=0):
            adapt_done = (rreg(c.fw_opt_done_addr) >> lane) & 1
        result[lane] = sd, rdy, adapt_done
    return result
def sig_det(lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        get_lane_mode(lane)
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
        sd_bit = rreg(c.rx_sd_addr, lane)
        #rdy = rreg(c.rx_rdy_addr, lane)
        result[lane] = sd_bit
    return result
def phy_rdy(lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        get_lane_mode(lane)
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
        #sd = rreg(c.rx_sd_addr, lane)
        rdy = rreg(c.rx_rdy_addr, lane)
        result[lane] = rdy
    return result
def adapt_done(lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        get_lane_mode(lane)
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg
        adapt_done=1
        if fw_loaded(print_en=0):
            adapt_done = (rreg(c.fw_opt_done_addr) >> lane) & 1
        result[lane] = adapt_done
    return result
def ppm(lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        get_lane_mode(lane)
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
        ppm = twos_to_int(rreg(c.rx_freq_accum_addr, lane), 11)
        result[lane] = ppm
    return result
    
def state(lane = None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        get_lane_mode(lane)
        c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
        result[lane] = rreg(c.rx_state_addr, lane)
    return result
    
def edge(edge1 = None, edge2 = None, edge3 = None, edge4 = None, lane = None):
    lanes = get_lane_list(lane)
    result = {}
    c = Pam4Reg
    for lane in lanes:
        if edge1 == edge2 == edge3 == edge4 == None:
            edge1_v = rreg(c.rx_edge1_addr, lane)
            edge2_v = rreg(c.rx_edge2_addr, lane)
            edge3_v = rreg(c.rx_edge3_addr, lane)
            edge4_v = rreg(c.rx_edge4_addr, lane)
            result[lane] = edge1_v, edge2_v, edge3_v, edge4_v
        else:
            if edge1 != None:
                wreg(c.rx_edge1_addr, edge1, lane)
            if edge2 != None:
                wreg(c.rx_edge2_addr, edge2, lane)
            if edge3 != None:
                wreg(c.rx_edge3_addr, edge3, lane)
            if edge4 != None:
                wreg(c.rx_edge4_addr, edge4, lane)
    else:
        if result != {}: return result
        
def sweep_param(addr = None, sweep_range = None, rst = 0, dir = 1, t = 1, lane = None):
    '''
    dir=1 means increasing direction first, dir=0 means decreasing direction first
    '''
    if lane==None: lane=gLane
    if type(lane) != int:
        print "*** This is a per-lane function"
        return
    get_lane_mode(lane)
    c = Pam4Reg if lane_mode_list[lane].lower() == 'pam4' else NrzReg  
    val_org = rreg(addr, lane)
    #cntr_org = rreg(c.rx_cntr_target_final_addr, lane)
    #wreg(c.rx_cntr_target_final_addr, 0x80, lane)
    options = {}
    if val_org in sweep_range:
        start = sweep_range.index(val_org)
    else:
        start = 0
    if dir == 1:
        range1 = sweep_range[start:len(sweep_range)]
        range2 = sweep_range[0:start]
        range2.reverse()
    else:
        range1 = sweep_range[0:start]
        range1.reverse()
        range2 = sweep_range[start:len(sweep_range)]
    for search_range in (range1, range2):
        #print search_range
        for val in search_range:
            #print val
            wreg(addr, val, lane)
            if rst == 1: lane_reset(lane = lane)
            rdy = sum(ready(lane = lane)[lane])
            cnt = 0
            while(rdy != 2):
                time.sleep(0.001)
                cnt += 1
                rdy = sum(ready(lane = lane)[lane])
                if cnt >= 1000: break
            else:
                time.sleep(t)
                options[val] = metric(lane = lane)
                #print options[val]
            # if moving in this direction makes link down, stop here
            if cnt >= 1000: break
    else:
        #print options
        best_val = max_metric(options)

    if gPrint:
        print "Value DAC EM   EM0(f1) EM1(f2) EM2(f3)     "
        for i in options.keys():
            print "%4d %2d %6.2f %6.2f %6.2f %6.2f "%(i, options[i][0][0], options[i][0][1], options[i][1][0], options[i][1][1], options[i][1][2]),
            if i == best_val:
                print "<<< Best Setting"
            else:
                print ""
    wreg(addr, best_val, lane)
    #wreg(rx_cntr_target_final_addr, cntr_org, lane)
    return best_val
    
def metric(repeat = 3, lane = None):
    '''
    use dac, eye margin and eye_pam4 as metric
    '''
    if lane==None: lane=gLane[0]
    if type(lane) != int:
        print "*** This is a per-lane function"
        return
    em = eye(lane=lane)[lane]
    dac_val = dac(lane=lane)[lane]
    em1, em2, em3 = eye_pam4(lane=lane)[lane] if lane_mode_list[lane] == 'pam4' else nrz_dfe(lane=lane)[lane]
    for i in range(repeat-1):
        em += eye(lane = lane)[lane][1]
        em_pam4 = eye_pam4(lane=lane)[lane] if lane_mode_list[lane] == 'pam4' else nrz_dfe(lane=lane)[lane]
        em1 += em_pam4[0]
        em2 += em_pam4[1]
        em3 += em_pam4[2]
    em = em/repeat
    em1 = em1/repeat
    em2 = em2/repeat
    em3 = em3/repeat
    return (dac_val,em), (em1, em2, em3)
    
def max_metric(options = None, lane = None):
    if lane == None: lane = gLane[0]
    if type(lane) != int:
        print "*** This is a per-lane function..."
        return
    keys = options.keys()
    keys.sort()
    best_val = keys[0]
    best_em = 0.0
    if lane_mode_list[lane] == 'pam4':
    # for pam4, use eye margin and pam4 three eyes to opt
        best_em_min = 0.0
        best_em_mean = 0.0
        for key in options.keys():
            #dac, em = options[key][0]
            em_pam4 = options[key][1]
            em_min = min(em_pam4)
            em_mean = sum(em_pam4)/len(em_pam4)
            if em_mean > best_em_mean and em_min >= best_em_min: #em >= best_em
                best_val = key
                #best_em = em
                best_em_min = em_min
                best_em_mean = em_mean
    else:
    # for nrz, use eye margin to opt
        for key in options.keys():
            em = options[key][0]
            #f1, f2, f3 = options[key][1]
            if em >= best_em:
                best_val = key
                best_em = em
    return best_val

def get_err(lane = None):
    
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        c = NrzReg if lane_mode_list[lane].lower() == 'nrz' else Pam4Reg
        # check if prbs checker is on
        checker = rreg(c.rx_prbs_checker_pu_addr, lane)
        rdy = sum(ready(lane)[lane])
        if checker == 0:
            print("***Lane %s PRBS checker is off***"%lane_name_list[lane])
            result[lane] = -1
        else:
            err = long(rreg(c.rx_err_cntr_msb_addr, lane)<<16) + rreg(c.rx_err_cntr_lsb_addr, lane)
            result[lane] = err
    else:
        if result != {}: return result

####################################################################################################
# 
# GET DFE Taps when FW not loaded
####################################################################################################
def sw_pam4_isi_2(b0=1, dir=0, isi_tap_range=range(0,9), t = 0.2, timer_s6 = 8, lane = None):
    if lane==None: lane=gLane[0]
    result={}
    if fw_loaded:
        #print("...sw_nrz_isi: Slice %d Lane %2d has FW Loaded. Exiting!"%(gSlice,lane))
        result[lane] = [-1]*len(isi_tap_range)
        return result
    if type(lane) != int:
        print "...sw_pam4_isi: This is a single-lane function"
        result[lane] = [-1]*len(isi_tap_range)
        return result
    c = Pam4Reg
    timer_meas_s6_org = rreg(c.rx_timer_meas_s6_addr, lane)    
    bp1_org = bp1(lane=lane)[lane][0:2]
    bp2_org = bp2(lane=lane)[lane][0:2]
    iter_s6_org = rreg(c.rx_iter_s6_addr, lane)
    mu_org = rreg(c.rx_mu_ow_addr, lane)
    mu_owen_org = rreg(c.rx_mu_owen_addr, lane)
    
    tap_list = []
    tap_list1 = [-2,-1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
    wreg(c.rx_fixed_patt_b0_addr, b0, lane)
    wreg(c.rx_fixed_patt_dir_addr, dir, lane)
    wreg(c.rx_timer_meas_s6_addr, timer_s6, lane)
    bp1(1,0x12,lane=lane)
    bp2(0,0,lane=lane)
    cnt = 1
    while True:
        if bp1(lane=lane)[lane][-1]: 
            mj = rreg(c.rx_mj_addr, lane)
            if mj == 0:
                break
            else:
                bp1(0,0,lane)
                sm_cont(lane)
                bp1(1,0x12,lane)
        else:
            cnt += 1
            if cnt > 5000:
                print "*** sw_pam4_isi: Timeout waiting for BP1 (1)"
                break
    wreg(c.rx_margin_patt_dis_owen_addr, 1, lane)
    wreg(c.rx_margin_patt_dis_ow_addr, 0, lane)
    wreg(c.rx_mu_ow_addr, 0, lane)
    wreg(c.rx_mu_owen_addr, 1, lane)
    wreg(c.rx_iter_s6_addr, 1, lane)
    bp1(0,0x12,lane=lane)
    sm_cont(lane=lane)

    for i in isi_tap_range:
        wreg(c.rx_fixed_patt_mode_addr, i, lane)
        time.sleep(t)
        bp1(1,0x12,lane=lane)
        cnt = 0
        while True:
            if bp1(lane=lane)[lane][-1]: break
            else:
                cnt += 1
                if cnt > 5000:
                    print "*** sw_pam4_isi: Timeout waiting for BP1 (2)"
                    break

        # read back fixed pattern margins (plus margin and minus margin)reg_addr1 = 0x36
        plus = rreg(c.rx_fixed_patt_plus_margin_addr, lane)
        minus = rreg(c.rx_fixed_patt_minus_margin_addr, lane)

        if (plus>2047): plus = plus - 4096
        if (minus>2047): minus = minus - 4096
        diff_margin = plus - minus 
        diff_margin_f = ((float(diff_margin & 0x0fff)/2048)+1)%2-1
        #if gPrint: print "%8d, %2d, %3d, %8d, %8d, %11d, %11.4f "  % (tap_list1[i], b0, dir, plus, minus, diff_margin, diff_margin_f)
        tap_list.append(diff_margin)

        bp1(0,0x12,lane=lane)
        sm_cont(lane=lane)

    bp1(1,0x12,lane=lane)
    cnt = 0
    while True:
        if bp1(lane=lane)[lane][-1]: break
        else:
            cnt += 1
            if cnt > 5000:
                print "*** sw_pam4_isi: Timeout waiting for BP1 (3)"
                break
    wreg(c.rx_timer_meas_s6_addr, timer_meas_s6_org, lane)
    wreg(c.rx_mu_ow_addr, mu_org, lane)
    wreg(c.rx_mu_owen_addr, mu_owen_org, lane)
    wreg(c.rx_iter_s6_addr, iter_s6_org, lane)
    wreg(c.rx_margin_patt_dis_owen_addr, 0, lane)
    wreg(c.rx_margin_patt_dis_ow_addr, 1, lane)    
    bp1(0,0x0,lane=lane)
    sm_cont(lane=lane)
    bp1(bp1_org[0], bp1_org[1], lane=lane)
    bp2(bp2_org[0], bp2_org[1], lane=lane)
    sm_cont(lane=lane)
    return tap_list
####################################################################################################
# 
# GET ISI Residual Taps
####################################################################################################
def sw_pam4_isi(b0=None, dir=0, isi_tap_range=range(0,9), lane=None, print_en=0):
    if lane==None: lane=gLane[0]
    result={}
    if fw_loaded:
        #print("...sw_nrz_isi: Slice %d Lane %2d has FW Loaded. Exiting!"%(gSlice,lane))
        result[lane] = [-1]*len(isi_tap_range)
        return result
    if type(lane) != int:
        print "...sw_pam4_isi: This is a single-lane function"
        result[lane] = [-1]*len(isi_tap_range)
        return result

        
    print_en2=0
    
    if b0==None:
        print_en=1
        b0=2
    else:
        print_en=0
        
    org1 = rreg(c.rx_margin_patt_dis_owen_addr, lane)
    org2 = rreg(c.rx_margin_patt_dis_ow_addr  , lane)
    org3 = rreg(c.rx_mu_ow_addr               , lane)
    org4 = rreg(c.rx_iter_s6_addr             , lane)
    org5 = rreg(c.rx_timer_meas_s6_addr       , lane)
    org6 = rreg(c.rx_bp1_en_addr              , lane)
    org7 = rreg(c.rx_bp1_st_addr              , lane)
    org8 = rreg(c.rx_bp2_en_addr              , lane)
    org9 = rreg(c.rx_bp2_st_addr              , lane)
    org10= rreg(c.rx_sm_cont_addr             , lane)
    bp1_org = bp1(lane=lane)[lane][0:2]
    
    # program registers for , b0, dir, pattern_p and pattern_n
    tap_list = []
    tap_saturated = []
    ffe_index_list = ['   ','Del','k1','k2','k3','k4','  ','  ','  ','  ','   ','   ','   ','   ','   ','   ','   ','   ','   ','   ','   ']
    #ffe_index_list = ['   ','k-1','k12','k3','k4','  ','  ','  ','  ','  ','   ','   ','   ','   ','   ','   ','   ','   ','   ','   ','   ']
    tap_index_list = ['f-2','f-1','f2' ,'f3','f4','f5','f6','f7','f8','f9','f10','f11','f12','f13','f14','f15','f16','f17','f18','f19','f20']
    wreg(c.rx_fixed_patt_b0_addr, b0, lane)
    wreg(c.rx_fixed_patt_dir_addr, dir, lane)
    wreg(c.rx_timer_meas_s6_addr, 8, lane)
    
    #### Clear BP2, Toggle BP1 and Continue to BP1 at state 0x12
    bp2(0,0x12,lane=lane)
    bp1(0,0x12,lane=lane)
    sm_cont_01(lane=lane)
    bp1(1,0x12,lane=lane)    

    wait_for_bp1_timeout = 0
    while True:
        if bp1(lane=lane)[lane][-1]: 
            #ma = rreg(c.rx_state_cntr_addr, lane) # wait for the whole 32-bit counter to be '0x00000000', not good
            ma = rreg(c.rx_mj_addr,lane) # wait for the lowest 2 bits to become '00'
            if ma == 0:
                break
            else:
                #print("0x%x,"%ma),
                bp1(0,0x12,lane)
                sm_cont_01(lane=lane)
                bp1(1,0x12,lane)
        else:
            wait_for_bp1_timeout += 1
            if wait_for_bp1_timeout > 5000:
                print("\nGet Tap Value FULL (1)***>> Timed out waiting for read_state_counter=0, before starting Getting Taps")
                if print_en2: print bp1(lane=lane)[lane][-1]
                bp2(0,0x12,lane=lane)
                bp1(0,0x12,lane=lane)
                break
                
                
    wreg(c.rx_margin_patt_dis_owen_addr, 1, lane)
    wreg(c.rx_margin_patt_dis_ow_addr, 0, lane)
    wreg(c.rx_mu_ow_addr, 0, lane)
    wreg(c.rx_iter_s6_addr, 1, lane)
    bp1(0,0x12,lane=lane)
    sm_cont_01(lane=lane)
    
    for i in isi_tap_range:
        wreg(c.rx_fixed_patt_mode_addr, i, lane)
        time.sleep(0.2)
        bp1(1,0x12,lane=lane)
        wait_for_bp1_timeout = 0
        while True:
            if bp1(lane=lane)[lane][-1]: break
            else:
                wait_for_bp1_timeout += 1
                if wait_for_bp1_timeout > 5000:
                    print("\nGet Tap Value FULL (2) ***>> Timed out waiting to reach BP1 for tap %d"%i)
                    bp2(0,0x12,lane=lane)
                    bp1(0,0x12,lane=lane)
                    break
        # read back fixed pattern margins (plus margin and minus margin)reg_addr1 = 0x36
        plus = rreg(c.rx_fixed_patt_plus_margin_addr, lane)
        minus = rreg(c.rx_fixed_patt_minus_margin_addr, lane)
    
        if (plus>2047): plus = plus - 4096
        if (minus>2047): minus = minus - 4096
        diff_margin = plus - minus 
        diff_margin_f = ((float(diff_margin & 0x0fff)/2048)+1)%2-1
        if print_en2: print "%3d,%2d,%2d,%5d,%5d,%5d,%8.4f "  % (i, b0, dir, plus, minus, diff_margin, diff_margin_f)
    
        if abs(plus) == 2048 or abs(minus) == 2048:
            print("\nGet Tap Value FULL ***>> Margin saturated to +/-2048 for tap %d"%i)
            diff_margin = 2222
            
        tap_list.append(diff_margin)
        bp1(0,0x12,lane=lane)
        sm_cont_01(lane=lane)
    
    bp1(1,0x12,lane=lane)
    wait_for_bp1_timeout = 0
    while True:
        if bp1(lane=lane)[lane][-1]: break
        else:
            wait_for_bp1_timeout += 1
            if wait_for_bp1_timeout > 5000:
                print("\nGet Tap Value FULL (3) ***>> Timed out waiting to reach BP1")
                break
    bp1(0,0x12,lane=lane)#debug
    sm_cont_01(lane=lane)#debug           
    wreg(c.rx_margin_patt_dis_owen_addr, org1 ,lane)
    wreg(c.rx_margin_patt_dis_ow_addr  , org2 ,lane)
    wreg(c.rx_mu_ow_addr               , org3 ,lane)
    wreg(c.rx_iter_s6_addr             , org4 ,lane)
    wreg(c.rx_timer_meas_s6_addr       , org5 ,lane)
    wreg(c.rx_bp1_en_addr              , org6 ,lane)
    wreg(c.rx_bp1_st_addr              , org7 ,lane)
    wreg(c.rx_bp2_en_addr              , org8 ,lane)
    wreg(c.rx_bp2_st_addr              , org9 ,lane)
    wreg(c.rx_sm_cont_addr             , org10,lane)
    sm_cont_01(lane=lane)
    
    #print 
    #rregBits(0x28,[13,9],1)
    if print_en:
        print("\n...SW Residual ISI Taps: Lane %s\n\n|"%lane_name_list[lane]),
        
        for i in isi_tap_range:
            print("%4s |"%ffe_index_list[i]),
        print("\n|"),
        [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin] = ffe_taps(lane=lane)[lane]
        delta_val = delta(lane=lane)[lane]
        print('     | %3d  | %4d | %4d | %4d | %4d |      |      |      |\n|' %(delta_val,ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin)),

        for i in isi_tap_range:
            print("%4s |"%tap_index_list[i]),
        print("\n|"),
        for i in range(len(tap_list)):
            print("%4s |"%tap_list[i]),
        print("\n")   
    else:
        return tap_list

Gettapvalue_full = sw_pam4_isi
Gettapvalue_pam4 = sw_pam4_isi_2
####################################################################################################
# 
# GET Residual ISI by the FW
####################################################################################################
def fw_pam4_isi(lane=None, print_en=0):
    isi_tap_range=range(0,16)
    lanes = get_lane_list(lane)

    result = {}
    for ln in lanes:
        get_lane_mode(ln)
        # Lane is in PAM4 mode, get the ISI taps
        if lane_mode_list[ln] == 'pam4' and phy_rdy(ln)[ln] == 1: 
            readout = BE_info_signed(ln, 10, 0, len(isi_tap_range))
        # Lane is in NRZ mode, or phy not ready, Skip
        else: 
            readout = [-1*ln]*len(isi_tap_range)
        result[ln] = readout
        
    return result
####################################################################################################
# 
# GET Residual ISI by the FW if loaded, or by software
####################################################################################################
def pam4_isi(lane=None,print_en=0):
    result = {}
    if fw_loaded:
        result = fw_pam4_isi(lane,print_en)
    else:
        result = sw_pam4_isi(lane,print_en)
    return result
####################################################################################################
# 
# GET Residual ISI by the FW if loaded, or by software
####################################################################################################
def nrz_isi(lane=None,print_en=0):
    result = {}
    if fw_loaded:
        result = sw_nrz_isi(lane,print_en)
    else:
        result = sw_nrz_isi(lane,print_en)
    return result
####################################################################################################
# 
# GET Residual ISI in NRZ or PAM4
####################################################################################################
def isi(lane=None,print_en=1):
    lanes = get_lane_list(lane)
    result = {}
    for ln in lanes:
        get_lane_mode(ln)
        if lane_mode_list[ln] == 'pam4':
            result[ln] = pam4_isi(ln,print_en)
        else: # NRZ
            result[ln] = nrz_isi(ln,print_en)
    
    if print_en:
        isi_tap_range=range(16)
        line_separator= "\n+----------------------------------------------------------------------------------------------------+"
        ffe_index_list = ['  ','Del','k1','k2','k3','k4','  ','  ','  ','  ','   ','   ','   ','   ','   ','   ','   ']
        tap_index_list = ['f-2','f-1','f2','f3','f4','f5','f6','f7','f8','f9','f10','f11','f12','f13','f14','f15','f16']
        print line_separator,
        if print_en==2:
            print("\n|   "),            
            for i in isi_tap_range:
                print("|%4s"%ffe_index_list[i]),
            print("|"),
        print("\n| Ln"),            
        for i in isi_tap_range:
            print("|%4s"%tap_index_list[i]),
        print("|"),
        print line_separator,

        for ln in lanes:
            if print_en==2:
                if lane_mode_list[ln] == 'pam4':
                    [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin] = ffe_taps(lane=ln)[ln]
                else:
                    [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin] = [0,0,0,0,0,0,0] 
                delta_val = delta(lane=ln)[ln]            
                print("\n             %3d  %4d  %4d  %4d  %4d                                                             |" %(delta_val,ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin)),
            print("\n| %2s"%lane_name_list[ln]),            
            for i in isi_tap_range:
                print(" %4d"%(result[ln][ln][i])),
            print("|"),
        print line_separator,
    else:
        return result

def channel_analyzer_pam4(dac_val = 0xf, ctle_val = 8, gain1_val = 22, gain2_val = 31, gain3_val = 8, lane = None):
    if lane==None: lane=gLane[0]
    if type(lane) != int:
        print "\nChannel Analyzer PAM4: *** This is a per-lane function"
        return 1,1,0
    if sig_det(lane)[lane]==0:
        print "\nChannel Analyzer PAM4: *** No Signal Detected"
        return 1,1,0
    bp1_org = bp1(lane = lane)[lane]
    bp2_org = bp2(lane = lane)[lane]
    ctle_map0_org = ctle_map(7, lane = lane)[lane]
    ctle_org = ctle(lane = lane)[lane]
    hf_ths_org = rreg(c.rx_hf_ths_addr, lane)
    of_ths_org = rreg(c.rx_of_ths_addr, lane)
    hf_period_org = rreg(c.rx_hf_period_addr, lane)
    of_period_org = rreg(c.rx_of_period_addr, lane)
    hf_cntr_target_org = rreg(c.rx_hf_cntr_target_addr, lane)
    of_cntr_upper_limit_org = rreg(c.rx_of_cntr_upper_limit_addr, lane)
    of_cntr_lower_limit_org = rreg(c.rx_of_cntr_lower_limit_addr, lane)
    updown_org = rreg(c.rx_theta_update_mode_addr, lane)
    ffe_org = ffe_taps(lane=lane)[lane]
    dc_gain_org = dc_gain(lane=lane)[lane]
    skef_org = skef(lane=lane)[lane]
    delta_org = delta(lane=lane)[lane]
    super_cal1_org = rreg(c.rx_mu_ow_addr, lane)
    super_cal_en_org = rreg(c.rx_mu_owen_addr, lane)
    
    wreg(c.rx_theta_update_mode_addr, 0, lane)
    edge_org = rreg(0xed, lane)
    f1o3_org = rreg(0x04, lane)
    wreg(0x04, 0xb0e9, lane)
    wreg(0xed, 0xcccc, lane)
    delta(0, lane)
    agc_degen_org = rreg(c.rx_agc_degen_addr, lane)
    wreg(c.rx_of_cntr_upper_limit_addr, 0xffff, lane)
    wreg(c.rx_of_cntr_lower_limit_addr, 0x0, lane)
    wreg(c.rx_hf_cntr_target_addr, 0xffff, lane)
    wreg(c.rx_of_period_addr, 14, lane)
    wreg(c.rx_hf_period_addr, 14, lane)
    dac(1, dac_val, lane)
    wreg(c.rx_mu_ow_addr, 0x0, lane)
    wreg(c.rx_mu_owen_addr, 0x1, lane)
    if ctle_val == 8:
        ctle_map(7, 7, 7, lane)
        wreg(c.rx_agc_degen_addr, 0x1, lane)
        ctle(7, lane)
        skef(0, 0, lane)
        ffe_taps(0x11, 0x11, 0x11, -0x11, 0x32, 0x30, lane)
        dc_gain(gain1_val, gain2_val, gain3_val, 1, lane)
    
    of_ths = 0x1f
    hf_ths = 0x1f
    of_f = 0
    hf_f = 0
    of_cnt3_f = 0
    of_cnt0_f = 0
    hf_cnt_f = 0
    hf_ths_f = 0
    of_ths_f = 0
    wreg(c.rx_of_ths_addr, of_ths, lane)
    wreg(c.rx_hf_ths_addr, hf_ths, lane)
    for i in range(8):
        wreg(c.rx_lane_rst_addr, 1, lane)
        if i > 0:
            wreg(c.rx_acal_start_owen_addr, 1, lane)
            wreg(c.rx_acal_start_ow_addr, 0, lane)
            wreg(c.rx_acal_clk_en_owen_addr, 1, lane)
            wreg(c.rx_acal_clk_en_ow_addr, 1, lane)
            wreg(c.rx_acal_done_owen_addr, 1, lane)
            wreg(c.rx_acal_done_ow_addr, 0, lane)        
        bp1(1,9,lane)
        bp2(0,0,lane)
        sm_cont(lane)
        wreg(c.rx_lane_rst_addr, 0, lane)
        wreg(c.rx_acal_done_owen_addr, 0, lane)
        wreg(c.rx_acal_start_owen_addr, 0, lane)
        wreg(c.rx_acal_clk_en_owen_addr, 0, lane)
        step = (i<7) and (0x10>>i) or 1
        timeout = 0
        while True:
            bp1_reached = bp1(lane=lane)[lane][-1]
            if bp1_reached:
                of_cnt0 = rreg(c.rx_cnt0_addr, lane)
                of_cnt3 = rreg(c.rx_cnt3_addr, lane)
                if of_cnt0 >= 1 and of_cnt3 >= 1:
                    if gDebugTuning: print "\nChannel Analyzer PAM4:  of_thre:%3d, cnt_3:%4d, cnt_0:%4d"%(of_ths, of_cnt3, of_cnt0),
                    of_cnt0_f = of_cnt0
                    of_cnt3_f = of_cnt3
                    of_ths_f = of_ths
                    if of_ths + step <= 0x3f:
                        of_ths += step
                else:
                    if gDebugTuning: print "\nChannel Analyzer PAM4: *of_thre:%3d, cnt_3:%4d, cnt_0:%4d"%(of_ths, of_cnt3, of_cnt0),
                    of_ths -= step
                wreg(c.rx_of_ths_addr, of_ths, lane)
                hf_cnt = rreg(c.rx_hf_cnt_addr, lane)
                if hf_cnt < 2:
                    if gDebugTuning:print "Miss:  hf_thre:%3d, cnt_hf:%6d"%(hf_ths, hf_cnt)
                    hf_ths -= step
                else:
                    if gDebugTuning:print "Hit :  hf_thre:%3d, cnt_hf:%6d"%(hf_ths, hf_cnt)
                    hf_ths_f = hf_ths
                    hf_cnt_f = hf_cnt
                    if hf_ths + step <= 0x3f:
                        hf_ths += step
                wreg(c.rx_hf_ths_addr, hf_ths, lane)            
                break
            else:
                timeout += 1
                if timeout > 5000: # was 5000, Alex 20180123
                    print "\nChannel Analyzer PAM4 *** Timed out waiting for BP1",
                    break
        
    #print dc_gain(lane=lane)[lane]
    ctle_map(7, ctle_map0_org[0], ctle_map0_org[1], lane = lane)
    ctle(ctle_org, lane = lane)
    dac(0,0,lane)
    dc_gain(dc_gain_org[0], dc_gain_org[1], dc_gain_org[2], dc_gain_org[3], lane)
    skef(skef_org[0], skef_org[1], lane)
    wreg(0x04, f1o3_org, lane)
    wreg(0xed, edge_org, lane)
    delta(delta_org, lane)
    wreg(c.rx_theta_update_mode_addr, updown_org, lane)
    ffe_taps(*ffe_org, lane=lane)
    wreg(c.rx_agc_degen_addr, agc_degen_org, lane)
    wreg(c.rx_hf_period_addr, hf_period_org, lane)
    wreg(c.rx_of_period_addr, of_period_org, lane)
    wreg(c.rx_hf_cntr_target_addr, hf_cntr_target_org, lane)
    wreg(c.rx_of_cntr_upper_limit_addr, of_cntr_upper_limit_org, lane)
    wreg(c.rx_of_cntr_lower_limit_addr, of_cntr_lower_limit_org, lane)
    wreg(c.rx_acal_start_owen_addr, 0, lane)
    wreg(c.rx_acal_clk_en_owen_addr, 0, lane)
    wreg(c.rx_acal_done_owen_addr, 0, lane)
    wreg(c.rx_hf_ths_addr, hf_ths_org, lane)
    wreg(c.rx_of_ths_addr, of_ths_org, lane)
    wreg(c.rx_mu_ow_addr, super_cal1_org, lane)
    wreg(c.rx_mu_owen_addr, super_cal_en_org, lane)
    bp1(0,0,lane)
    bp2(0,0,lane)
    sm_cont(lane)
    bp1(bp1_org[0], bp1_org[1], lane = lane)
    bp2(bp2_org[0], bp2_org[1], lane = lane)
    if hf_ths_f != 0:
        ratio = float(of_ths_f)/float(hf_ths_f)
    else:
        ratio = 0.0
    return of_ths_f, hf_ths_f, ratio
    

    
####################################################################################################
## NRZ specific functions
####################################################################################################
def nrz_dfe(lane = None):
    lanes = get_lane_list(lane)

    c = NrzReg
    result = {}
    for lane in lanes:
        f1 = twos_to_int(rreg(c.rx_f1_addr, lane),7)
        f2 = twos_to_int(rreg(c.rx_f2_addr, lane),7)
        f3 = twos_to_int(rreg(c.rx_f3_addr, lane),7)
        result[lane] = f1, f2, f3
    return result
    
def bb_nrz(en = None, lane = None):
    lanes = get_lane_list(lane)
    c = NrzReg
    result = {}
    for lane in lanes:
        if en != None:
            wreg(c.rx_bb_en_addr, en, lane) # bb_en
            wreg(c.rx_delta_adapt_en_addr, 1-en, lane) # delta_adapt_en
        else:
            bb = rreg(c.rx_bb_en_addr, lane)
            delta_en = rreg(c.rx_delta_adapt_en_addr, lane)
        result[lane] = bb, delta_en
    else:
        if result != {}: return result
        
def channel_analyzer_nrz(dac_val = 0xf, ctle_val = 8, gain1_val = 2, gain2_val = 31, gain3_val = 8, timer = 14, lane = None):
    if lane==None: lane=gLane[0]
    if type(lane) != int:
        print "\nChannel Analyzer NRZ: *** This is a per-lane function"
        return 1,1,0
    if sig_det(lane)[lane]==0:
        print "\nChannel Analyzer NRZ: *** No Signal Detected"
        return 1,1,0
    c = NrzReg
    bp1_org = bp1(lane = lane)[lane]
    bp2_org = bp2(lane = lane)[lane]
    ctle_map0_org = ctle_map(7, lane = lane)[lane]
    ctle_org = ctle(lane = lane)[lane]
    hf_ths_org = rreg(c.rx_hf_ths_addr, lane)
    of_ths_org = rreg(c.rx_of_ths_addr, lane)
    hf_period_org = rreg(c.rx_hf_period_addr, lane)
    of_period_org = rreg(c.rx_of_period_addr, lane)
    hf_cntr_target_org = rreg(c.rx_hf_cntr_target_addr, lane)
    of_cntr_upper_limit_org = rreg(c.rx_of_cntr_upper_limit_addr, lane)
    of_cntr_lower_limit_org = rreg(c.rx_of_cntr_lower_limit_addr, lane)
    dc_gain_org = dc_gain(lane=lane)[lane]
    skef_org = skef(lane=lane)[lane]
    agc_degen_org = rreg(Pam4Reg.rx_agc_degen_addr, lane)
    updown_org = rreg(c.rx_theta_update_mode_addr, lane)
    ffe_org = ffe_taps(lane=lane)[lane]
    dc_gain_org = dc_gain(lane=lane)[lane]
    delta_org = delta(lane=lane)[lane]
    
    wreg(c.rx_theta_update_mode_addr, 0, lane)
    edge_org = rreg(0xed, lane)
    wreg(0xed, 0x8888, lane)
    delta(0, lane)

    wreg(c.rx_of_cntr_upper_limit_addr, 0xffff, lane)
    wreg(c.rx_of_cntr_lower_limit_addr, 0x0, lane)
    wreg(c.rx_hf_cntr_target_addr, 0xffff, lane)
    wreg(c.rx_of_period_addr, timer, lane)
    wreg(c.rx_hf_period_addr, timer, lane)
    wreg(c.rx_dac_owen_addr, 0x1, lane)
    wreg(c.rx_dac_ow_addr, dac_val, lane)
    if ctle_val == 8:
        ctle_map(7,7,7, lane)
        wreg(Pam4Reg.rx_agc_degen_addr, 0x1, lane)
        ctle(7, lane)
        skef(0, 0, lane)
        dc_gain(gain1_val, gain2_val, gain3_val, 0, lane)

    of_ths = 0x1f
    hf_ths = 0x1f
    of_f = 0
    hf_f = 0
    of_cnt_f = 0
    hf_cnt_f = 0
    wreg(c.rx_of_ths_addr, of_ths, lane)
    wreg(c.rx_hf_ths_addr, hf_ths, lane)

    #main loop
    # of setup
    wreg(c.rx_of_cntr_lower_limit_addr, 0xffff, lane)
    bp1(1,8,lane)
    bp2(1,9,lane)
    #bp2(1,8,lane)
    #sm_cont(lane)
    wreg(c.rx_lane_rst_addr, 0, lane)
    wreg(c.rx_lane_rst_addr, 1, lane)
    wreg(c.rx_acal_start_owen_addr, 1, lane)
    wreg(c.rx_acal_start_ow_addr, 0, lane)
    wreg(c.rx_acal_clk_en_owen_addr, 1, lane)
    wreg(c.rx_acal_clk_en_ow_addr, 1, lane)
    wreg(c.rx_acal_done_owen_addr, 1, lane)
    wreg(c.rx_acal_done_ow_addr, 0, lane)
    wreg(c.rx_lane_rst_addr, 0, lane)
    wreg(c.rx_acal_start_owen_addr, 0, lane)
    wreg(c.rx_acal_clk_en_owen_addr, 0, lane)
    wreg(c.rx_acal_done_owen_addr, 0, lane)
    #wreg(c.rx_sm_cont_addr, 1, lane)
    #wreg(c.rx_lane_rst_addr, 0, lane)    
    start_time = time.time()
    for t in range(1,9):     
        wait_for_bp1_timeout=0
        step =  (t<7) and (0x40>>(t+1)) or 1
        while True: # OF While-loop, First wait for states 8 or 9
            bp1_reached=bp1(lane=lane)[lane][-1]
            bp2_reached=bp2(lane=lane)[lane][-1]
            if bp1_reached or bp2_reached:
                of_cnt = rreg(c.rx_patt_cnt_addr, lane)
                if(gDebugTuning):
                    t1 = time.time() - start_time
                    st = state(lane=lane)[lane]
                    print "\nChannel Analyzer NRZ: %10.5f:    of_thre: %04x, of_cnt: %04x, %d" % (t1, of_ths, of_cnt, st),
                if (of_cnt>1) or bp2_reached: # if non-zero cnt or state=9
                    #if init_zero_cnt==1: # this is a false 0 cnt
                    of_f = of_ths
                    of_cnt_f = of_cnt
                    if of_ths + step <= 0x3f:
                        of_ths = of_ths + step
                        wreg(c.rx_of_ths_addr, of_ths, lane)
                else:                           # if zero cnt and state=8
                    # if of_ths - step >= 0: 
                        # of_ths = of_ths - step
                    # else:
                        # of_ths = 0
                    # wreg(c.rx_of_ths_addr, of_ths, lane)
                    of_ths = of_ths - step
                    wreg(c.rx_of_ths_addr, of_ths, lane)
                    #if t==1: init_zero_cnt=1 # if first time state 8, cnt=0, not sure if true or flase 0 cnt
                break
            else:
                wait_for_bp1_timeout+=1
                if wait_for_bp1_timeout>20: # was 100, Alex 20180123
                    if(gDebugTuning):print "\nChannel Analyzer NRZ:  ***>> Timed out waiting for BP1=8 or BP2=9 (timeout counter = %d)"%wait_for_bp1_timeout,
                    #break
        # hf setup
        wreg(c.rx_of_cntr_lower_limit_addr, 0x0, lane)
        if t == 8:
            bp1(1,0xb,lane)
            bp2(1,0xc,lane)
        else:
            bp1(1,0xb,lane)
            bp2(1,0xc,lane)
        wait_for_bp1_timeout=0
        sm_cont(lane)
        while True:
            bp1_reached = bp1(lane=lane)[lane][-1]
            bp2_reached = bp2(lane=lane)[lane][-1]
            t1 = time.time() - start_time
            if bp1_reached or bp2_reached:
                hf_cnt = rreg(c.rx_patt_cnt_addr, lane)
                if hf_cnt < 1:                
                    if(gDebugTuning):print " %10.5f:       Miss: hf_thre: %04x, st: %04x, hf_cnt: %04x" % (t1, hf_ths, rreg(c.rx_state_addr, lane), hf_cnt),
                    hf_ths = hf_ths - step
                    wreg(c.rx_hf_ths_addr, hf_ths, lane)
                else:
                    if(gDebugTuning):print " %10.5f:        Hit: hf_thre: %04x, st: %04x, hf_cnt: %04x" % (t1, hf_ths, rreg(c.rx_state_addr, lane), hf_cnt),
                    hf_f = hf_ths
                    hf_cnt_f = hf_cnt               
                    if hf_ths + step <= 0x3f:
                        hf_ths = hf_ths + step
                        wreg(c.rx_hf_ths_addr, hf_ths, lane)
                break
            else:
                wait_for_bp1_timeout+=1
                if wait_for_bp1_timeout>20:
                    if(gDebugTuning):print("\nChannel Analyzer NRZ: ***>> Timed out waiting for BP1=0xB, BP2=0xC (timeout counter = %d)"%wait_for_bp1_timeout),
                    st = rreg(c.rx_state_addr, lane)
                    if(gDebugTuning): print ("State: %d"%st)
                    break
        if t != 8:
            bp1(1,8,lane)
            bp2(1,9,lane)
            wreg(c.rx_of_cntr_lower_limit_addr, 0xffff, lane)
            sm_cont(lane)

    ctle_map(7, ctle_map0_org[0], ctle_map0_org[1], lane = lane)
    ctle(ctle_org, lane = lane)
    skef(skef_org[0], skef_org[1], lane)
    wreg(Pam4Reg.rx_agc_degen_addr, agc_degen_org, lane)
    dac(0,lane)
    dc_gain(dc_gain_org[0], dc_gain_org[1], dc_gain_org[2], dc_gain_org[3], lane)
    wreg(c.rx_hf_period_addr, hf_period_org, lane)
    wreg(c.rx_of_period_addr, of_period_org, lane)
    wreg(c.rx_hf_cntr_target_addr, hf_cntr_target_org, lane)
    wreg(c.rx_of_cntr_upper_limit_addr, of_cntr_upper_limit_org, lane)
    wreg(c.rx_of_cntr_lower_limit_addr, of_cntr_lower_limit_org, lane)
    wreg(c.rx_acal_start_owen_addr, 0, lane)
    wreg(c.rx_acal_clk_en_owen_addr, 0, lane)
    wreg(c.rx_acal_done_owen_addr, 0, lane)
    wreg(c.rx_theta_update_mode_addr, updown_org, lane)
    ffe_taps(*ffe_org, lane=lane)
    dc_gain(*dc_gain_org, lane=lane)
    delta(delta_org, lane=lane)
    wreg(0xed, edge_org, lane)
    bp1(0,0,lane)    
    bp2(0,0,lane)
    sm_cont(lane)    
    #bp1(bp1_org[0], bp1_org[1], lane = lane)
    #bp2(bp2_org[0], bp2_org[1], lane = lane)
    if hf_f != 0:
      ratio = float(of_f)/hf_f
    else:
      ratio = 0.0
    return [of_f, hf_f, ratio]
    
def ffe_pu(en = None, lane = None):
    lanes = get_lane_list(lane)
    c = Pam4Reg
    result = {}
    for lane in lanes:
        if en == None:
            pu1 = rreg(c.rx_ffe_k1_pu_addr, lane)
            pu2 = rreg(c.rx_ffe_k2_pu_addr, lane)
            pu3 = rreg(c.rx_ffe_k3_pu_addr, lane)
            pu4 = rreg(c.rx_ffe_k4_pu_addr, lane)
            result[lane] = pu1 & pu2 & pu3 & pu4
        else:
            wreg(c.rx_ffe_k1_pu_addr, en, lane)
            wreg(c.rx_ffe_k2_pu_addr, en, lane)
            wreg(c.rx_ffe_k3_pu_addr, en, lane)
            wreg(c.rx_ffe_k4_pu_addr, en, lane)
    else:
        if result != {}: return result
    
def ffe_taps (k1=None, k2=None, k3=None, k4=None, s1=None, s2=None, sf=None, lane=None):

    lanes = get_lane_list(lane)
    c = Pam4Reg
    result = {}
    for lane in lanes:
        if k1!=None:
            pol1 = 1 if k1<0 else 0
            k11 = abs(k1)
            rx_ffe_k1_gray_msb = Bin_Gray(k11 >> 4)
            rx_ffe_k1_gray_lsb = Bin_Gray(k11&0xf)
            rx_ffe_k1_gray = ((rx_ffe_k1_gray_msb << 4) + rx_ffe_k1_gray_lsb)&0xFF
            wreg(c.rx_ffe_k1_msb_addr, rx_ffe_k1_gray_msb, lane)
            wreg(c.rx_ffe_k1_lsb_addr, rx_ffe_k1_gray_lsb, lane)
            wreg(c.rx_ffe_pol1_addr, pol1, lane)
        if k2!=None:
            pol2 = 1 if k2<0 else 0
            k22 = abs(k2)
            rx_ffe_k2_gray_msb = Bin_Gray(k22 >> 4)
            rx_ffe_k2_gray_lsb = Bin_Gray(k22&0xf)
            rx_ffe_k2_gray = ((rx_ffe_k2_gray_msb << 4) + rx_ffe_k2_gray_lsb)&0xFF
            wreg(c.rx_ffe_k2_msb_addr, rx_ffe_k2_gray_msb, lane)
            wreg(c.rx_ffe_k2_lsb_addr, rx_ffe_k2_gray_lsb, lane)
            wreg(c.rx_ffe_pol2_addr, pol2, lane)
        if k3!=None:
            pol3 = 1 if k3<0 else 0
            k33 = abs(k3)
            rx_ffe_k3_gray_lsb = Bin_Gray(k33&0xf)
            rx_ffe_k3_gray_msb = Bin_Gray(k33 >> 4)
            rx_ffe_k3_gray = ((rx_ffe_k3_gray_msb << 4) + rx_ffe_k3_gray_lsb)&0xFF
            wreg(c.rx_ffe_k3_msb_addr, rx_ffe_k3_gray_msb, lane)
            wreg(c.rx_ffe_k3_lsb_addr, rx_ffe_k3_gray_lsb, lane)
            wreg(c.rx_ffe_pol3_addr, pol3, lane)
        if k4!=None:
            pol4 = 1 if k4<0 else 0
            k44 = abs(k4)
            rx_ffe_k4_gray_lsb = Bin_Gray(k44&0xf)
            rx_ffe_k4_gray_msb = Bin_Gray(k44 >> 4)
            rx_ffe_k4_gray = ((rx_ffe_k4_gray_msb << 4) + rx_ffe_k4_gray_lsb)&0xFF
            wreg(c.rx_ffe_k4_msb_addr, rx_ffe_k4_gray_msb, lane)
            wreg(c.rx_ffe_k4_lsb_addr, rx_ffe_k4_gray_lsb, lane)
            wreg(c.rx_ffe_pol4_addr, pol4, lane)
        if s2!=None:
            rx_ffe_s2_gray_lsb = Bin_Gray(s2&0x0f)
            rx_ffe_s2_gray_msb = Bin_Gray((s2 >> 4)&0x0f)
            rx_ffe_s2_gray = ((rx_ffe_s2_gray_msb << 4) + rx_ffe_s2_gray_lsb)&0xFF
            wreg(c.rx_ffe_s2_msb_addr, rx_ffe_s2_gray_msb, lane)
            wreg(c.rx_ffe_s2_lsb_addr, rx_ffe_s2_gray_lsb, lane)
        if s1!=None:
            rx_ffe_s1_gray_lsb = Bin_Gray(s1&0x0f)
            rx_ffe_s1_gray_msb = Bin_Gray((s1 >> 4)&0x0f)
            rx_ffe_s1_gray = ((rx_ffe_s1_gray_msb << 4) + rx_ffe_s1_gray_lsb)&0xFF
            wreg(c.rx_ffe_s1_msb_addr, rx_ffe_s1_gray_msb, lane)
            wreg(c.rx_ffe_s1_lsb_addr, rx_ffe_s1_gray_lsb, lane)
            
        if sf!=None:
            rx_ffe_sf_gray_lsb = Bin_Gray(sf&0x0f)
            rx_ffe_sf_gray_msb = Bin_Gray((sf >> 4)&0x0f)
            rx_ffe_sf_gray = ((rx_ffe_sf_gray_msb << 4) + rx_ffe_sf_gray_lsb)&0xFF
            wreg(c.rx_ffe_sf_msb_addr, rx_ffe_sf_gray_msb, lane)
            wreg(c.rx_ffe_sf_lsb_addr, rx_ffe_sf_gray_lsb, lane)
            
        if k1==None and k2==None and k3==None and k4==None and s1==None and s2==None and sf==None:
            pol1 = rreg(c.rx_ffe_pol1_addr, lane)
            rx_ffe_k1_msb = Gray_Bin(rreg(c.rx_ffe_k1_msb_addr, lane))
            rx_ffe_k1_lsb = Gray_Bin(rreg(c.rx_ffe_k1_lsb_addr, lane))
            rx_ffe_k1_bin = (1-2*pol1)*(((rx_ffe_k1_msb << 4) + rx_ffe_k1_lsb)&0xFF)
        
            pol2 = rreg(c.rx_ffe_pol2_addr, lane)
            rx_ffe_k2_msb = Gray_Bin(rreg(c.rx_ffe_k2_msb_addr,  lane))
            rx_ffe_k2_lsb = Gray_Bin(rreg(c.rx_ffe_k2_lsb_addr,  lane))
            rx_ffe_k2_bin = (1-2*pol2)*(((rx_ffe_k2_msb << 4) + rx_ffe_k2_lsb)&0xFF)

            pol3 = rreg(c.rx_ffe_pol3_addr, lane)
            rx_ffe_k3_msb = Gray_Bin(rreg(c.rx_ffe_k3_msb_addr,  lane))
            rx_ffe_k3_lsb = Gray_Bin(rreg(c.rx_ffe_k3_lsb_addr,  lane))
            rx_ffe_k3_bin = (1-2*pol3)*(((rx_ffe_k3_msb << 4) + rx_ffe_k3_lsb)&0xFF)
            
            pol4 = rreg(c.rx_ffe_pol4_addr, lane)
            rx_ffe_k4_msb = Gray_Bin(rreg(c.rx_ffe_k4_msb_addr, lane))
            rx_ffe_k4_lsb = Gray_Bin(rreg(c.rx_ffe_k4_lsb_addr, lane))
            rx_ffe_k4_bin = (1-2*pol4)*(((rx_ffe_k4_msb << 4) + rx_ffe_k4_lsb)&0xFF)
            
            rx_ffe_s1_msb = Gray_Bin(rreg(c.rx_ffe_s1_msb_addr,  lane))
            rx_ffe_s1_lsb = Gray_Bin(rreg(c.rx_ffe_s1_lsb_addr,  lane))
            rx_ffe_s1_bin = ((rx_ffe_s1_msb << 4) + rx_ffe_s1_lsb)&0xFF
            
            rx_ffe_s2_msb = Gray_Bin(rreg(c.rx_ffe_s2_msb_addr,  lane))
            rx_ffe_s2_lsb = Gray_Bin(rreg(c.rx_ffe_s2_lsb_addr,  lane))
            rx_ffe_s2_bin = ((rx_ffe_s2_msb << 4) + rx_ffe_s2_lsb)&0xFF
            
            rx_ffe_sf_msb = Gray_Bin(rreg(c.rx_ffe_sf_msb_addr,  lane))
            rx_ffe_sf_lsb = Gray_Bin(rreg(c.rx_ffe_sf_lsb_addr,  lane))
            rx_ffe_sf_bin = ((rx_ffe_sf_msb << 4) + rx_ffe_sf_lsb)&0xFF
            
            result[lane] = rx_ffe_k1_bin, rx_ffe_k2_bin, rx_ffe_k3_bin, rx_ffe_k4_bin, rx_ffe_s1_bin, rx_ffe_s2_bin, rx_ffe_sf_bin
        
    else:
        if result != {}: return result
        
def ctle_search(lane = None, t = 0.02):
    if lane == None: lane = gLane[0]
    if type(lane) != int:
        print "\nCTLE Search PAM4: *** This is a per-lane function"
        return
    # settings
    cntr_target_org = rreg(c.rx_cntr_target_final_addr, lane)
    
    #wreg(c.rx_cntr_target_final_addr, 0x2000, lane)
    #time.sleep(0.1)
    #wreg(c.rx_cntr_target_final_addr, 0x1000, lane)
    
    # channel analyzer and EQ1
    # time1 = time.time()
    # of, hf, ratio = channel_analyzer_pam4(lane=lane)
    # if ratio == 0:
        # print "*** Channel Analyzer Error..."
        # if gPrint: print "of = %d, hf = %d, ratio = %.3f"%(of, hf, ratio)
        # return
    # ctle_init = channel_eq1(of, hf, ratio, lane=lane) # it sets ctle and other initial settings
    # time2 = time.time()
    # if gPrint: print "Channel Analyzer spent %.3fs"%(time2-time1)
    # # lane reset
    # time1 = time.time()
    # lane_reset(lane=lane)
    # rdy = wait_rdy(lane=lane, t = 10)[lane]
    # st = state(lane)[lane]
    # time2 = time.time()
    # if sum(rdy) != 2:
        # st = state(lane=lane)
        # if gDebugTuning: print "*** Initial linkup failed, %d %.3f..."%(st, ratio)
        # return
    # else:
        # if gDebugTuning: print "Reset spent %.3fs"%(time2-time1)
    # time.sleep(t)
    # EQ2
    ctle_init = ctle(lane=lane)[lane]
    time1 = time.time()
    ctle_f = channel_eq2(ctle_init, t, lane = lane)
    time2 = time.time()
    if ctle_f != -1:
        ctle(ctle_f, lane=lane)
        if gDebugTuning: print "\nCTLE Search PAM4: spent  %.3fs"%(time2-time1),
    else:
        print "\nCTLE Search PAM4: *** EQ2 failed, Initial CTLE:%d"%(ctle_init),
        ctle(ctle_init, lane=lane)
        return -1
    # done
    wreg(c.rx_cntr_target_final_addr, cntr_target_org, lane)
    if gDebugTuning:
        #print "of = %d, hf = %d, ratio = %.3f, ctle_init = %d, ctle_f = %d"%(of, hf, ratio, ctle_init, ctle_f)
        print '\nCTLE Search PAM4: ctle_init = %d, ctle_final = %d'%(ctle_init, ctle_f),
    return ctle_f
########################################################################    
def ctle_fine_search(lane=None, mode=1):
    ctle_init = ctle(lane=lane)[lane]
    start_time = time.time()
    if mode==1: tap_range = range(2,6)
    else: tap_range = range(8,15)
    tap_value = [0]*len(tap_range)
    iter = 5
    for i in range(iter):
        tap_list = sw_pam4_isi (isi_tap_range=tap_range,lane=lane)
        tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(gDebugTuning):print('\nCTLE Search:'),
        if(gDebugTuning):print(["%6.1f"%x for x in tap_list]),
    if(gDebugTuning):print('\nCTLE Search:'),
    if(gDebugTuning):print(["%6.1f"%x for x in tap_value]),
    
    ctle_f = ctle_init
    ih = 0
    il = 0
    addall = 0
    for tap in tap_value:
        if tap<0: il +=1
        elif tap>0: ih +=1
        addall += tap
    addall = addall/len(tap_value)
    if(gDebugTuning):print '\nCTLE Search: Average: %f, NumOfPos: %d, NumOfNeg: %d' % (addall, ih, il),
    if mode==1:
        if (addall<-48)&(il>=(len(tap_value)/2))&(tap_value[0]<-80)&(tap_value[1]<-20):
            if (ctle_init<7): ctle_f = ctle_init+1
        if (addall>48)&(ih>=(len(tap_value)/2))&(tap_value[0]>80)&(tap_value[1]>20):
            if (ctle_init>1): ctle_f = ctle_init-1
    else:
        if (addall<-18)&(il>=(len(tap_value)/2)): 
            if (ctle_init<7): ctle_f = ctle_init+1
        if (addall>18)&(ih>=(len(tap_value)/2)):
            if (ctle_init>1): ctle_f = ctle_init-1

    if ctle_f != -1:
        ctle(ctle_f, lane=lane)
        if gDebugTuning: print "\nCTLE fine Search PAM4: spent  %.3fs"%(time.time()-start_time),
    else:
        print "\nCTLE fine Search PAM4: *** EQ2 failed, Initial CTLE:%d"%(ctle_init),
        ctle(ctle_init, lane=lane)
        return -1
        
    if(gDebugTuning):print ('\nCTLE Search: CTLE value= %d' % ctle_f),
    #if(gDebugTuning):print ('\nCTLE Search: OptTime: %2.2f, '%(time.time()-start_time))
    return ctle_f

####################################################################################################     
def channel_eq1(of = 63, hf = 63, ratio = 1.0, lane = None):
    lanes = get_lane_list(lane)
        
    if ratio <1.10:                # DIRECT LOOPBACK  ############## USE SR TABLE #################
        tx_taps(+1,-4,17,0,0,lane=lanes)
        ctle(7,lane=lanes)
        delta(5,lane=lanes)
        f13(0,lanes)
        edge(8,8,8,8,lane=lanes)
        skef(1,2,lane=lanes)
        dc_gain(1,4,8,12, lane=lanes) # worse BER comapred to 1,4,1,0
        ffe_taps(0x33,0x00,0x00,-0x01,0x01,0x01,lane=lanes) # external loopback  
        ctle_init = 7
    elif ratio <1.15: # Artek 0-10%, SR, less than 10dB
        tx_taps(+2,-8,17,0,0,lane=lanes)
        ctle(6, lane=lanes)
        delta(-10,lane=lanes)
        f13(4,lane=lanes)
        edge(8,8,8,8,lane=lanes)
        skef(1,2,lane=lanes)
        dc_gain(1,20,8,8, lane=lanes) # worse BER comapred to 1,4,1,0
        ffe_taps(0x44,0x00,0x00,-0x11,0x01,0x01,lane=lanes) # external loopback 
        ctle_init = 6
    elif ratio <1.34:  # Artek 10%-30%, MR - 1.57
        tx_taps(+2,-8,17,0,0,lane=lanes)
        ctle(5, lane=lanes)
        delta(-10,lane=lanes)
        f13(3,ext_cable_lanes)
        edge(8,8,8,8,lane=lanes)
        skef(1,3,lane=lanes)
        dc_gain(1,25,8,8, lane=lanes) # worse BER comapred to 1,4,1,0
        ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,lane=lanes) # external loopback
        ctle_init = 5
    elif ratio <1.58:  # Artek 40%-60%, MR, less than 40% 
        tx_taps(+4,-12,17,0,0,lane=lanes)
        ctle(4, lane=lanes)
        delta(-11,lane=lanes)
        f13(4,lane=lanes)
        edge(8,8,8,8,lane=lanes)
        skef(1,3,lane=lanes)
        dc_gain(8,31,8,8, lane=lanes) # worse BER comapred to 1,4,1,0
        ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,lane=lanes) # external loopback  
        ctle_init = 4
    elif ratio <1.63:  # Artek 40%-60%, MR, less than 40% ############### SWITCH TO LR TABLE #################
        tx_taps(+5,-16,17,0,0,lane=lanes)
        ctle(3, lane=lanes)
        delta(-5,lane=lanes)
        f13(5,lane=lanes)
        edge(8,8,8,8,lane=lanes)
        skef(1,3,lane=lanes)
        dc_gain(20,31,8,8, lane=lanes) # worse BER comapred to 1,4,1,0
        ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,lane=lanes) # external loopback  
        ctle_init = 3
    elif ratio <1.66:  # Artek 40%-60%, MR, less than 40% 
        tx_taps(+5,-16,17,0,0,lane=lanes)
        ctle(4, lane=lanes)
        delta(-6,lane=lanes)
        f13(5,lane=lanes)
        edge(8,8,8,8,lane=lanes)
        skef(1,3,lane=lanes) 
        dc_gain(25,31,8,2, lane=lanes) # worse BER comapred to 1,4,1,0
        ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,lane=lanes) # external loopback  
        ctle_init = 4
    elif ratio <1.9:  # Artek 40%-60%, MR, less than 40% 
        tx_taps(+5,-16,17,0,0,lane=lanes)
        ctle(4, lane=lanes)
        delta(-8,lane=lanes)
        f13(6,lane=lanes)
        edge(8,8,8,8,lane=lanes)
        skef(1,3,lane=lanes)
        dc_gain(35,31,8,2, lane=lanes) # worse BER comapred to 1,4,1,0
        ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,lane=lanes) # external loopback                
        ctle_init = 4
    elif ratio <2.1:  # Artek 60%-80%, LR, less than 
        tx_taps(+5,-16,17,0,0,lane=lanes)
        ctle(3, lane=lanes)
        delta(-10,lane=lanes)
        f13(7,lane=lanes)
        edge(8,8,8,8,lane=lanes)
        skef(1,3,lane=lanes)
        dc_gain(45,31,12,1, lane=lanes) # worse BER comapred to 1,4,1,0
        ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,lane=lanes) # external loopback 
        ctle_init = 3
    elif ratio <2.3:  # Artek 60%-80%, LR, less than 
        tx_taps(+5,-16,17,0,0,lane=lanes)
        ctle(2, lane=lanes)
        delta(-10,lane=lanes)
        f13(9,lane=lanes)
        edge(8,8,8,8,lane=lanes)
        skef(1,3,lane=lanes)
        dc_gain(60,31,12,1, lane=lanes) # worse BER comapred to 1,4,1,0
        ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,lane=lanes) # external loopback                 
        ctle_init = 2
    else: #if chan_est >=2.00: # Artek 80%-100%, LR,
        tx_taps(+5,-16,17,0,0,lane=lanes)
        ctle(1, lane=lanes)
        delta(-10,lane=lanes)
        f13(10,lane=lanes)
        edge(8,8,8,8,lane=lanes)
        skef(1,3,lane=lanes)
        dc_gain(75,31,15,1, lane=lanes) # worse BER comapred to 1,4,1,0
        ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,lane=lanes) # external loopback 
        ctle_init = 1
    return ctle_init
    
def channel_eq2(ctle_init = 7, t = 0.02, lane = None):
    if lane == None:
        lane = gLane[0]
    if type(lane) != int:
        print "\nChannel EQ2 PAM4: *** This is a per-lane function..."
        return
        
    ctle_search_map = { 7: [7, 6, 5],
                        6: [6, 5, 4, 5, 6, 7],
                        5: [5, 4, 3, 4, 5, 6, 7],
                        4: [4, 3, 2, 3, 4, 5, 6],
                        3: [3, 2, 1, 2, 3, 4, 5],
                        2: [2, 1, 0, 1, 2, 3, 4],
                        1: [1, 0, 1, 2, 3],
                        0: [0, 1, 2],
                        }
    ctle_option = ctle_search_map[ctle_init]
    metric_option = {}
    for ctle_v in ctle_option:
        ctle_pre = ctle(lane=lane)[lane]
        ctle(ctle_v, lane=lane)
        # check if link down, set ctle back
        if ready(lane)[lane] != (1,1): 
            ctle(ctle_pre, lane=lane)
            continue
        elif ctle_v in metric_option.keys():
            continue
        else:
            # need a smart reset here
            lane_reset_2(lane)
            time.sleep(t)
            em = eye(lane=lane)[lane]
            em_pam4 = eye_pam4(lane=lane)[lane]
            metric_option[ctle_v] = em, em_pam4
    if metric_option != {}:
        ctle_f = max_metric(metric_option, lane)
        if gDebugTuning:
            for i in metric_option.keys():
                print i, metric_option[i]
    else:
        ctle_f = -1

    return ctle_f   

def lane_reset_2(lane = None):
    lanes = get_lane_list(lane)
    for lane in lanes:
        em_pam4 = eye_pam4(lane=lane)[lane]
        tmp = [i <= 5.0 for i in em_pam4]
        if True in tmp:
            lane_reset(lane)
            wait_rdy(lane)
#################################################
# tgt_cntr(lane=11)
# tgt_cntr('high',lane=11)
# tgt_cntr_nrz('low', lane=11)           
#################################################
def cntr_tgt_nrz(tgt_val=None, lane = None):
    lanes = get_lane_list(lane)


    final_tgt_list=[]
    for lane in lanes:
        tgt_val_list=[0x002,0x004,0x008,0x010,0x020,0x040,0x080,0x100]

        if lane_mode_list[lane] != 'nrz':
            lane_mode_list[lane] = 'nrz'
        c = NrzReg

        if tgt_val!=None:
            cntr_target_org = rreg(c.rx_cntr_target_addr, lane) # current setting for this lane
            if tgt_val=='low':#> cntr_target_org: 
                tgt_val_list.reverse()

            for tgt in tgt_val_list:
                wreg(c.rx_cntr_target_addr, tgt, lane); time.sleep(.001)
                #print("\nCNTR TGT: 0x%03X"%tgt),

        final_tgt_list.append(rreg(c.rx_cntr_target_addr, lane))

    return final_tgt_list
            
def ctle_search_nrz(lane = None, t = 0.02):
    if lane == None:
        lane = gLane[0]
    if type(lane) != int:
        print "\nCTLE Search NRZ: *** This is a per-lane function"
        return
    if lane_mode_list[lane] != 'nrz':
        lane_mode_list[lane] = 'nrz'
    c = NrzReg
    # settings
    cntr_target_org = rreg(c.rx_cntr_target_addr, lane)
    wreg(c.rx_cntr_target_addr, 0x100, lane)
    time.sleep(t)
    wreg(c.rx_cntr_target_addr, 0x80, lane)
    time.sleep(t)
    wreg(c.rx_cntr_target_addr, 0x40, lane)
    time.sleep(t)
    wreg(c.rx_cntr_target_addr, 0x20, lane)
    time.sleep(t)
    wreg(c.rx_cntr_target_addr, 0x10, lane)
    time.sleep(t)
    wreg(c.rx_cntr_target_addr, 0x8, lane)
    time.sleep(t)
    wreg(c.rx_cntr_target_addr, 0x4, lane)
    time.sleep(t)
    wreg(c.rx_cntr_target_addr, 0x2, lane)
        
    ctle_init = ctle(lane=lane)[lane]
    time1 = time.time()
    ctle_f = channel_eq2_nrz(ctle_init, t, lane = lane)
    time2 = time.time()
    if ctle_f != -1:
        ctle(ctle_f, lane=lane)
        if gDebugTuning: print "\nCTLE Search NRZ: Fine tuning spent %.3fs"%(time2-time1),
    else:
        print "\nCTLE Search NRZ: *** Fine tuning failed, %d..."%(ctle_init),
        ctle(ctle_init, lane=lane)
        return -1
    # done
    wreg(c.rx_cntr_target_addr, cntr_target_org, lane)
    if gDebugTuning:
        #print "of = %d, hf = %d, ratio = %.3f, ctle_init = %d, ctle_f = %d"%(of, hf, ratio, ctle_init, ctle_f)
        print '\nCTLE Search NRZ: ctle_init = %d, ctle_f = %d'%(ctle_init, ctle_f),
    return ctle_f

def channel_eq2_nrz(ctle_init = 7, t = 0.02, lane = None):
    if lane == None:
        lane = gLane[0]
    if type(lane) != int:
        print "\nChannel EQ2 NRZ: *** This is a per-lane function...",
        return
        
    ctle_search_map = { 7: [7, 6, 5],  # 6,5,6,7
                        6: [6, 5, 6, 7],
                        5: [5, 4, 5, 6],
                        4: [4, 3, 4, 5],
                        3: [3, 2, 3, 4],
                        2: [2, 1, 2, 3],
                        1: [1, 0, 1, 2],
                        0: [0, 1],
                        }
    ctle_option = ctle_search_map[ctle_init]
    metric_option = {}
    for ctle_v in ctle_option:
        ctle_pre = ctle(lane=lane)[lane]
        ctle(ctle_v, lane=lane)
        # check if link down, set ctle back
        if ready(lane)[lane] != (1,1): 
            ctle(ctle_pre, lane=lane)
            continue
        elif ctle_v in metric_option.keys():
            continue
        else:
            time.sleep(t)
            em = eye(lane=lane)[lane]
            dfe = nrz_dfe(lane=lane)[lane]
            metric_option[ctle_v] = em, dfe
    if metric_option != {}:
        ctle_f = max_metric(metric_option, lane)
        if gDebugTuning:
            for i in metric_option.keys():
                print i, metric_option[i]
    else:
        ctle_f = -1

    return ctle_f
    
####################################################################################################  
# Slice 0, A0-A3: PAM4 loopback, B0-B7: NRZ to Falcon
# Falcon should be in Functional mode
####################################################################################################
def fec_config(lane=None):

    
    lane=8 # B0 in NRZ mode
    wreg([0x0F1,[3]],1,lane) # turn on RX FEC Clock
    wreg([0x0FB,[0]],1,lane) # turn on TX FEC Clock
    
    lane=0 # A0 in PAM4 mode
    wreg([0x0F1,[3]],1,lane) # turn on RX FEC Clock
    wreg([0x0FB,[0]],1,lane) # turn on TX FEC Clock
    
    wreg([0x9857,[0]],1) # turn on A-side FEC
    wreg([0x9857,[4]],1) # turn on B-side FEC
    
    wreg(0x5880,0x0F00)    # reset B-side FEC
    wreg(0x5880,0x0B00)    # release reset of B-side TX FEC
    wreg(0x5880,0x0A00)    # release reset of B-side RX FEC
    
    FEC_Lock_status = rreg(0x50C9,[15,8])
    if FEC_Lock_status == 0x4F:
        print("\n...B-side KR4 FEC is locked\n") # here put Falcon in Functional mode
    else:
        print("\n***B-side KR4 FEC NOT locked <<<\n")

    wreg(0x4880,0x0F00)    # reset A-side FEC
    wreg(0x4880,0x0B00)    # release reset of A-side TX
    wreg(0x4880,0x0A00)    # release reset of A-side RX
    
    FEC_Lock_status = rreg(0x40C9,[15,8])
    if FEC_Lock_status == 0x4F:
        print("\n...A-side KP4 FEC is locked\n") 
    else:
        print("\n***A-side KP4 FEC NOT locked <<<\n")
    
    # B-side NRZ LT Enabled (B0-B7)
    for lane in range(0,8):
        wreg(0x3800+0x100*lane,0x0002)    # B-side NRZ LT Training toggled
        wreg(0x3000+0x100*lane,0x0000)    # B-side NRZ LT Training toggled
        wreg(0x3000+0x100*lane,0x0002)    # B-side NRZ LT Training enabled

    # A-side PAM4 LT Disabled (A0-A3)
    for lane in range(0,5):
        wreg(0x3800+0x100*lane,0x0002)    # A-side PAM4 LT Training toggled
        wreg(0x3800+0x100*lane,0x0000)    # A-side PAM4 LT Training Disable

    # B-side NRZ TX reset (B0-B7)
    wreg(0x5880,0x0A00)    # reset B-side TX FEC
    wreg(0x5880,0x0E00)    # reset of B-side TX FEC
    wreg(0x5880,0x0A00)    # release reset of B-side TX FEC
        
    # Falcon commands, just for reference (port=1):
    # hex(chip.TCMRd(0xC0004,1))       # expected 0xcf03, i.e. PCS locked
    # chip.TCMWr(0xC0200,4,1)          # Enable KR4
    # hex(chip.TCMRd(0xC0200,1))       # expected value 0x8004 (bit 2 = status)    
    # hex(chip.TCMRd(0x04004,1))       # PCS link status, expected value? 
    # chip.TCMWr(0x40000, 0xa050, 1)   #???
    # chip.TCMWr(0xc0200, 4, 1)        #???
    # chip.TCMWr(0xc0200, 4, 0)        #???
    # chip.TCMWr(0xc0200, 4, 1)        #???
    
####################################################################################################  
# FEC Parameters Readback functions
####################################################################################################
#           [FEC_A0,FEC_A1,FEC_A2,FEC_A2, FEC_B0,FEC_B1,FEC_B2,FEC_B3]
fec_class = [0x4000,0x4100,0x4200,0x4300, 0x5000,0x5100,0x5200,0x5300]
fec_class2= [0x4400,0x4500,0x4600,0x4700, 0x5400,0x5500,0x5600,0x5700]
fec_class3= [0x4800,0x4900,0x4a00,0x4b00, 0x5800,0x5900,0x5a00,0x5b00]

####################################################################################################

def fec_status_en(index):
    counter = rregBits(0x9857, [index])
    return counter
    
def fec_status_traffic_gen_en(index): #wreg(0x48f0,0x1095)
    status = rregBits(fec_class3[index]+0xF0,[7])
    return status
        
def fec_status_aligned_rx(index):
    status = rregBits(fec_class[index]+0xc9,[14])
    return status

def fec_status_ampsLock(index):
    status = rregBits(fec_class[index]+0xc9,[11,8])
    return status

def fec_status_uncorrected(index):
    counter_lower = rregBits(fec_class[index]+0xcc,[15,0])
    counter_upper = rregBits(fec_class[index]+0xcd,[15,0])
    counter = (counter_upper<<16)+counter_lower
    return counter

def fec_status_corrected(index):
    counter_lower = rregBits(fec_class[index]+0xca,[15,0])
    counter_upper = rregBits(fec_class[index]+0xcb,[15,0])
    counter = (counter_upper<<16)+counter_lower
    return counter

def fec_status_rx_fifo_cnt(index):
    counter = rregBits(fec_class3[index]+0x9f,[10,0])
    return counter
def fec_status_rx_fifo_min(index):
    counter = rregBits(fec_class3[index]+0xa0,[10,0])
    return counter
def fec_status_rx_fifo_max(index):
    counter = rregBits(fec_class3[index]+0xa1,[10,0])
    return counter
def fec_status_tx_fifo_cnt(index):
    counter = rregBits(fec_class3[index]+0x9c,[10,0])
    return counter
def fec_status_tx_fifo_min(index):
    counter = rregBits(fec_class3[index]+0x9d,[10,0])
    return counter
def fec_status_tx_fifo_max(index):
    counter = rregBits(fec_class3[index]+0x9e,[10,0])
    return counter
def fec_status_pre_fec_ber(fec_idx):

    if chip_rev == 1.0:
        print ("*** fec_status_pre_fec_ber() This function is not supported in Baldeagle A0 ***\n")
        return
        
    fec_base_addr = fec_class3[fec_idx]
    cw_size = 5440 if fec_idx < 4 else 5280
    
    wreg(fec_base_addr+0x63,0x8002) # freeze the counters bit[1]=1, turn the clock on for reading counters bit[4]=0
    time.sleep(.010)
    total_errors_hi   = rreg(fec_base_addr+0x7f)
    total_errors_mid  = rreg(fec_base_addr+0x7e)
    total_errors_lo   = rreg(fec_base_addr+0x7d)
 
    total_cw_hi  = rreg(fec_base_addr+0x5f)
    total_cw_mid = rreg(fec_base_addr+0x5e)
    total_cw_lo  = rreg(fec_base_addr+0x5d) 
   
    total_errors = long( (total_errors_hi << 32) + (total_errors_mid<<16) + (total_errors_lo) )
    total_cw     = long( (total_cw_hi     << 32) + (total_cw_mid    <<16) + (total_cw_lo    ) )
    ber = float(total_errors)/float(total_cw)/float(cw_size)
    
    #print ("FEC %d (base: 0x%04X) Pre-FEC BER:"%(fec_idx,fec_base_addr))
    #print ("Errors : %04X_%04X_%04X = %11d bits"%(total_errors_hi,total_errors_mid,total_errors_lo,total_errors))
    #print ("CWs    : %04X_%04X_%04X = %11d CWs"%(total_cw_hi,total_cw_mid,total_cw_lo, total_cw))
    if total_errors == 0 and total_cw != 0:
        ber = 0.0 # print ("BER    : 0")
    elif total_cw == 0:
        ber = -1.0 #print ("BER    : ERROR. CW Count =0 !!!")
    else:
        ber = float(total_errors)/float(total_cw)/float(cw_size)
        #print ("BER    : %2.2e"%(float(total_errors)/float(total_cw)/float(cw_size)))
    
    wreg(fec_base_addr+0x63,0x8000) # unfreeze the counters, bit[1]=0, 
    time.sleep(0.010)
    wreg(fec_base_addr+0x63,0x8001) # clear the counters for next time, bit[0]=1
    time.sleep(0.010)
    wreg(fec_base_addr+0x63,0x8000) # clear the counters for next time, bit[0]=0
    #time.sleep(0.010)
    #wreg(fec_base_addr+0x63,0x8010) # turn the clock off to save power, bit[4]=1
    return total_errors, total_cw, ber
    
####################################################################################################  
# Checks the status of all 8 FECs
#
# Returns: A list of eight FEC Status flags, one for each FEC_IDX 0 to 7
#
#         FEC Status = [FECA0,FECA1,FECA2,FECA3, FECB0,FECB1,FECB2,FECB3]
#
#         FEC Status[idx] =  0: If FEC is not powered up (unused or never configured)
#         FEC Status[idx] =  1: If FEC was able to LOCK (pass)
#         FEC Status[idx] = -1: If FEC was not able to lock (fail)
#
####################################################################################################
def fec_status(print_en=True, fifo_check_en=False):
    length_1 = 22
    length = 17
    
    if fifo_check_en: # [Acceptable range for each FIFO counter]
        rx_fec_fifo_min_limit = [354,  965]
        rx_fec_fifo_max_limit = [354,  965]
        rx_fec_fifo_del_limit = [  0,  514]
        
        tx_fec_fifo_min_limit = [320,  931]
        tx_fec_fifo_max_limit = [320,  931]
        tx_fec_fifo_del_limit = [  0,  514]
    else:
        rx_fec_fifo_min_limit = [  1, 1279]
        rx_fec_fifo_max_limit = [  1, 1279]
        rx_fec_fifo_del_limit = [  1, 1279]
        
        tx_fec_fifo_min_limit = [  1, 1279]
        tx_fec_fifo_max_limit = [  1, 1279]
        tx_fec_fifo_del_limit = [  1, 1279]
        
        
    data={}
    for index in range(8):
        data[index]=[]
        data[index].append(fec_status_aligned_rx(index))
        data[index].append(fec_status_ampsLock(index))
        data[index].append(fec_status_uncorrected(index))
        data[index].append(fec_status_corrected(index))
        data[index].append(fec_status_rx_fifo_min(index))
        data[index].append(fec_status_rx_fifo_cnt(index))
        data[index].append(fec_status_rx_fifo_max(index))
        data[index].append(fec_status_tx_fifo_min(index))
        data[index].append(fec_status_tx_fifo_cnt(index))
        data[index].append(fec_status_tx_fifo_max(index))
        data[index].append(fec_status_en(index))

    rx_adapt_done=[]

    aligned=[]
    ampslock=[]
    uncorrected=[]
    corrected=[]
    
    rx_fifo_min=[]
    rx_fifo_cnt=[]
    rx_fifo_max=[]
    rx_fifo_del=[]
    
    tx_fifo_min=[]
    tx_fifo_cnt=[]
    tx_fifo_max=[]
    tx_fifo_del=[]
    
    fec_en=[]
    fec_status=[]
    fec_statistics=[]; idx=0
    
    # Check all lanes' RX Adaptation Status
    rx_adapt_done_lane = [0]*(16+4)
    for ln in range(16):
        rx_adapt_done_lane[ln] = (rreg(c.fw_opt_done_addr) >> ln) & 1
           
    for index in range(8): # 4 A-FECs and 4 B-FECs
        # This FEC lanes' RX Adapt status
        if index < 4: # Adapt status of the A side lanes, related to this FEC
            rx_adapt_done.append([])
            if(rreg([0x9858,  [5,4]])== 2): # Check FEC_A2 Lane Mapping. If [5:4]=2, it means FEC_A2 is connected to lanes A2/A3, else A4/A5
                rx_adapt_done[index].append(rx_adapt_done_lane[index])     # adapt flag for lane  0 or 2
                rx_adapt_done[index].append(rx_adapt_done_lane[index+1])   # adapt flag for lane  1 or 3
            else:
                rx_adapt_done[index].append(rx_adapt_done_lane[index*2])   # adapt flag for lane  0 or 4
                rx_adapt_done[index].append(rx_adapt_done_lane[index*2+1]) # adapt flag for lane  1 or 5
            
        else:         # Adapt status of the B side lanes, related to this FEC
            rx_adapt_done.append([])
            rx_adapt_done[index].append(rx_adapt_done_lane[index*2])   # adapt flag for lane  8 or 12
            rx_adapt_done[index].append(rx_adapt_done_lane[index*2+1]) # adapt flag for lane  9 or 13
            rx_adapt_done[index].append(rx_adapt_done_lane[index*2+2]) # adapt flag for lane 10 or 14
            rx_adapt_done[index].append(rx_adapt_done_lane[index*2+3]) # adapt flag for lane 11 or 15        
        # This FEC status
        aligned.append(data[index][0])
        ampslock.append(data[index][1])
        uncorrected.append(data[index][2])
        corrected.append(data[index][3])
        rx_fifo_min.append(data[index][4])
        rx_fifo_cnt.append(data[index][5])
        rx_fifo_max.append(data[index][6])
        rx_fifo_del.append(rx_fifo_max[index] - rx_fifo_min[index])
        tx_fifo_min.append(data[index][7])
        tx_fifo_cnt.append(data[index][8])
        tx_fifo_max.append(data[index][9])
        tx_fifo_del.append(tx_fifo_max[index] - tx_fifo_min[index])
        fec_en.append(data[index][10])
        fec_status.append(1)
        
        # Return values for this FEC
        if fec_en[index]:
            fec_statistics.append([])
            fec_statistics[idx].append(rx_adapt_done[index])
            fec_statistics[idx].append(format(ampslock[index],'04b'))
            fec_statistics[idx].append(aligned[index])
            fec_statistics[idx].append(corrected[index])
            fec_statistics[idx].append(uncorrected[index])
            fec_statistics[idx].append(rx_fifo_min[index])
            fec_statistics[idx].append(rx_fifo_max[index])
            fec_statistics[idx].append(rx_fifo_del[index])
            fec_statistics[idx].append(tx_fifo_min[index])
            fec_statistics[idx].append(tx_fifo_max[index])
            fec_statistics[idx].append(tx_fifo_del[index])
            idx+=1
        
    num_fec_en = fec_en.count(1)
    #for testing
    #num_fec_en=8# = fec_en.count(1)
    #fec_en=[1,1,1,1,1,1,1,1]
    separator='\n |'
    for i in range(length_1+(length)*num_fec_en):
        separator+='-'
    separator += '|'   
    
    if print_en:
        print separator,
        print '\n |  Parameters          |',
        for index in range(8):
            if fec_en[index]:
                fec_type = 'A'   if index<4 else 'B'
                macro    = index if index<4 else index-4
                print('    FEC_%s[%d]   |'%(fec_type,macro)),

    if print_en: print separator,

    if print_en: print '\n | RX Lanes Adapt Done  |',
    for index in range(8):
        if fec_en[index]:
            flag='<<' if (0 in rx_adapt_done[index]) else '  '
            if flag=='<<': fec_status[index]=-1
            if print_en and index < 4: print("        %d,%d %s |"%(rx_adapt_done[index][0],rx_adapt_done[index][1],flag)),
            if print_en and index > 3: print("    %d,%d,%d,%d %s |"%(rx_adapt_done[index][0],rx_adapt_done[index][1],rx_adapt_done[index][2],rx_adapt_done[index][3],flag)),
            
    if print_en: print '\n | RX FEC AmLck 0,1,2,3 |',
    for index in range(8):
        if fec_en[index]:
            flag='<<' if (ampslock[index] != 0xF) else '  '
            if flag=='<<': fec_status[index]=-1
            if print_en: print("    %d,%d,%d,%d %s |"%(ampslock[index]>>0&1,ampslock[index]>>1&1,ampslock[index]>>2&1,ampslock[index]>>3&1,flag)),

    if print_en: print '\n | RX FEC Aligned       |',
    for index in range(8):
        if fec_en[index]:
            flag='<<' if (aligned[index]) != 1 else '  '
            if flag=='<<': fec_status[index]=-1
            if print_en: print(" %10d %s |"%(aligned[index],flag)),
        else:
            fec_status[index]=0
            
    if print_en: print '\n | RX FEC Corr   (cw)   |',
    for index in range(8):
        if fec_en[index]:
            if index < 4: flag='  ' if corrected[index] == 0 else '  '
            if index >=4: flag='<<' if corrected[index] != 0 else '  '
            if flag=='<<': fec_status[index]=-1
            if print_en: print(" %10d %s |"%(corrected[index],flag)),

    if print_en: print '\n | RX FEC Uncorr (cw)   |',
    for index in range(8):
        if fec_en[index]:
            flag='<<' if uncorrected[index] != 0 else '  '
            if flag=='<<': fec_status[index]=-1
            if print_en: print(" %10d %s |"%(uncorrected[index],flag)),

    if print_en: print separator,

    if print_en: print '\n | RX FEC FIFO Min      |',
    for index in range(8):
        if fec_en[index]:
            flag='<<' if not (rx_fec_fifo_min_limit[0] <= rx_fifo_min[index] <= rx_fec_fifo_min_limit[1]) else '  '
            if flag=='<<': fec_status[index]=-1
            if print_en: print(" %10d %s |"%(rx_fifo_min[index],flag)),
    if print_en: print '\n | RX FEC FIFO Cnt      |',
    for index in range(8):
        if fec_en[index]:
            flag='<<' if not (rx_fifo_min[index] <= rx_fifo_cnt[index] <= rx_fifo_max[index]) else '  '
            #if flag=='<<': fec_status[index]=-1 # do not fail for live count value
            if print_en: print(" %10d %s |"%(rx_fifo_cnt[index],flag)),
    if print_en: print '\n | RX FEC FIFO Max      |',
    for index in range(8):
        if fec_en[index]:
            flag='<<' if not (rx_fec_fifo_max_limit[0] <= rx_fifo_max[index] <= rx_fec_fifo_max_limit[1]) else '  '
            if flag=='<<': fec_status[index]=-1
            if print_en: print(" %10d %s |"%(rx_fifo_max[index],flag)),
    if print_en: print '\n | RX FEC FIFO Delta    |',            
    for index in range(8):
        if fec_en[index]:
            flag='<<' if not (rx_fec_fifo_del_limit[0] <= rx_fifo_del[index] <= rx_fec_fifo_del_limit[1]) else '  '
            if flag=='<<': fec_status[index]=-1
            if print_en: print(" %10d %s |"%(rx_fifo_del[index],flag)),

    if print_en: print separator,
    
    if print_en: print '\n | TX FEC FIFO Min      |',
    for index in range(8):
        if fec_en[index]:
            flag='<<' if not (tx_fec_fifo_min_limit[0] <= tx_fifo_min[index] <= tx_fec_fifo_min_limit[1]) else '  '
            if flag=='<<': fec_status[index]=-1
            if print_en: print(" %10d %s |"%(tx_fifo_min[index],flag)),
    if print_en: print '\n | TX FEC FIFO Cnt      |',
    for index in range(8):
        if fec_en[index]:
            flag='<<' if not (tx_fifo_min[index] <= tx_fifo_cnt[index] <= tx_fifo_max[index]) else '  '
            #if flag=='<<': fec_status[index]=-1 # do not fail for live count value
            if print_en: print(" %10d %s |"%(tx_fifo_cnt[index],flag)),
    if print_en: print '\n | TX FEC FIFO Max      |',
    for index in range(8):
        if fec_en[index]:
            flag='<<' if not (tx_fec_fifo_max_limit[0] <= tx_fifo_max[index] <= tx_fec_fifo_max_limit[1]) else '  '
            if flag=='<<': fec_status[index]=-1
            if print_en: print(" %10d %s |"%(tx_fifo_max[index],flag)),
    if print_en: print '\n | TX FEC FIFO Delta    |',
    for index in range(8):
        if fec_en[index]:
            flag='<<' if not (tx_fec_fifo_del_limit[0] <= tx_fifo_del[index] <= tx_fec_fifo_del_limit[1]) else '  '
            if flag=='<<': fec_status[index]=-1
            if print_en: print(" %10d %s |"%(tx_fifo_del[index],flag)),

    if print_en: print separator,

    if print_en: print '\n | Overall FEC STATUS   |',
    for index in range(8):
        if fec_en[index]:
            fec_overall_status='    LOCK     ' if fec_status[index]==1 else '*** FAIL *** '
            if print_en: print(" %12s |"%(fec_overall_status)),
             
    if print_en: 
        print separator,
    else: 
        return fec_status, fec_statistics
##################################################################################################
#
# def fec_hist(hist_time = 5, print_en=True, fecs = [0,2,4,6] )
#
##################################################################################################
# Collects FEC Correctable Error Count Histogram over a period of time
# for each FEC block selected
#
# Returns tuple of all FECs statistical information (if print_en=False):
#   
# {FEC_IDX:[DataRate,Time, AM Bits, TotalUncorrCnt, TotalCorrCnt, Bin0,...,Bin15]
#   {0: [53.125,   10, 1111, 0, 157, 151, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
#    2: [53.125,   10, 1111, 0,  30,  24, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
#    4: [25.78125, 10, 1111, 0,   0,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
#    6: [25.78125, 10, 1111, 0,   0,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
#
# Prints out histogram per FEC (if print_en=True):
#
# FEC Histogram (100 sec)
# +------------------------------------------------------------------------------------------+
# |  Parameters          |     FEC_A[0]   |     FEC_A[2]   |     FEC_B[0]   |     FEC_B[2]   |
# +------------------------------------------------------------------------------------------+
# | RX FEC AmLck 0,1,2,3 |        1111    |        1111    |        1111    |        1111    |
# | RX FEC Corr   (cw)   |        1706    |         211    |           0    |           0    |
# | RX FEC Uncorr (cw)   |           0    |           0    |           0    |           0    |
# +------------------------------------------------------------------------------------------+
# | RX FEC Bin  1        |        1648    |         193    |           0    |           0    |
# | RX FEC Bin  2        |          37    |          14    |           0    |           0    |
# | RX FEC Bin  3        |          12    |           4    |           0    |           0    |
# | RX FEC Bin  4        |           6    |           0    |           0    |           0    |
# | RX FEC Bin  5        |           1    |           0    |           0    |           0    |
# | RX FEC Bin  6        |           1    |           0    |           0    |           0    |
# | RX FEC Bin  7        |           1    |           0    |           0    |           0    |
# | RX FEC Bin  8        |           0    |           0    |           0    |           0    |
# | RX FEC Bin  9        |           0    |           0    |           0    |           0    |
# | RX FEC Bin 10        |           0    |           0    |           0    |           0    |
# | RX FEC Bin 11        |           0    |           0    |           0    |           0    |
# | RX FEC Bin 12        |           0    |           0    |           0    |           0    |
# | RX FEC Bin 13        |           0    |           0    |           0    |           0    |
# | RX FEC Bin 14        |           0    |           0    |           0    |           0    |
# | RX FEC Bin 15        |           0    |           0    |           0    |           0    |
# +------------------------------------------------------------------------------------------+
# / dsh >>> fec_hist(3600)
#
# FEC Histogram (3600 sec)
# +------------------------------------------------------------------------------------------+
# |  Parameters          |     FEC_A[0]   |     FEC_A[2]   |     FEC_B[0]   |     FEC_B[2]   |
# +------------------------------------------------------------------------------------------+
# | RX FEC AmLck 0,1,2,3 |        1111    |        1111    |        1111    |        1111    |
# | RX FEC Corr   (cw)   |       60416    |       11302    |           0    |           0    |
# | RX FEC Uncorr (cw)   |           0    |           0    |           0    |           0    |
# +------------------------------------------------------------------------------------------+
# | RX FEC Bin  1        |       58821    |       10029    |           0    |           0    |
# | RX FEC Bin  2        |         986    |         836    |           0    |           0    |
# | RX FEC Bin  3        |         382    |         285    |           0    |           0    |
# | RX FEC Bin  4        |         197    |         141    |           0    |           0    |
# | RX FEC Bin  5        |          15    |           7    |           0    |           0    |
# | RX FEC Bin  6        |          13    |           4    |           0    |           0    |
# | RX FEC Bin  7        |           2    |           0    |           0    |           0    |
# | RX FEC Bin  8        |           0    |           0    |           0    |           0    |
# | RX FEC Bin  9        |           0    |           0    |           0    |           0    |
# | RX FEC Bin 10        |           0    |           0    |           0    |           0    |
# | RX FEC Bin 11        |           0    |           0    |           0    |           0    |
# | RX FEC Bin 12        |           0    |           0    |           0    |           0    |
# | RX FEC Bin 13        |           0    |           0    |           0    |           0    |
# | RX FEC Bin 14        |           0    |           0    |           0    |           0    |
# | RX FEC Bin 15        |           0    |           0    |           0    |           0    |
# +------------------------------------------------------------------------------------------+
#
##################################################################################################
def fec_hist(hist_time = 5, print_en=True, fecs = [0,2,4,6] ):
    if chip_rev == 1.0:
        print ("*** fec_hist() function is not supported in Babbage A0 ***\n")
        return
    
    fec_class1= [0x4000,0x4100,0x4200,0x4300, 0x5000,0x5100,0x5200,0x5300]
    fec_class2= [0x4400,0x4500,0x4600,0x4700, 0x5400,0x5500,0x5600,0x5700]
    fec_class3= [0x4800,0x4900,0x4a00,0x4b00, 0x5800,0x5900,0x5a00,0x5b00]

    result={}
    bins=range(0,17)

    #### Start the FEC Counters
    for fec_idx in fecs:
        wregBits(fec_class3[fec_idx]+0x63, [0], 1) # clear error counter
        wregBits(fec_class3[fec_idx]+0x63, [0], 0)
        wregBits(fec_class3[fec_idx]+0x63, [1], 0) # unfreeze the counter
    
    #### Wait for FEC Stat Collection Time
    time.sleep(hist_time)
    
    #### Stop the FEC Counters and Histograms
    for fec_idx in fecs:        
        wregBits(fec_class3[fec_idx]+0x63, [1], 1) # freeze the counter
        
    #### Now process the FEC Counters and Histograms
    for fec_idx in fecs:
        result[fec_idx]=[]
        cw_size = 5440 if fec_idx < 4 else 5280
        data_rate = 53.125 if fec_idx < 4 else 25.78125
        bits_transferred = float((hist_time*data_rate*pow(10,9)))  
               
        ampslock = fec_status_ampsLock(fec_idx)
        total_uncorr_cnt= (rreg(fec_class1[fec_idx] +0xcd)<<16) + rreg(fec_class1[fec_idx] +0xcc) # read hist uncorr count
        total_corr_cnt  = (rreg(fec_class1[fec_idx] +0xcb)<<16) + rreg(fec_class1[fec_idx] +0xca) # read hist corr count
        result[fec_idx].append(data_rate)  # fec_idx][0]
        result[fec_idx].append(hist_time) # fec_idx][1]
        result[fec_idx].append((ampslock>>3&1)*1000 + (ampslock>>3&1)*100 + (ampslock>>1&1)*10 + (ampslock>>0&1) )  # fec_idx][2]
        result[fec_idx].append(total_uncorr_cnt) # fec_idx][3]
        result[fec_idx].append(total_corr_cnt) # fec_idx][4]
        
        corr_cnt_per_bin = [0]*17
        corr_ber_per_bin = [0]*17
        for bin in bins: # fec_idx][6++]
            wregBits(fec_class3[fec_idx]+0x60, [4,0], bin) # select hist bin
            corr_cnt_per_bin[bin] = (rreg(fec_class3[fec_idx]+0x62)<<16) + rreg(fec_class3[fec_idx]+0x61) # read hist corr count
            if bin == 0:
                wregBits(fec_class3[fec_idx]+0x60, [4,0], bin+17)
                corr_cnt_per_bin[bin] += (rreg(fec_class3[fec_idx]+0x61)<<32)
                total_cw_transferred = corr_cnt_per_bin[0] + total_corr_cnt + total_uncorr_cnt 
                result[fec_idx].append(total_cw_transferred) # fec_idx][5]

            result[fec_idx].append(corr_cnt_per_bin[bin]) # fec_idx][6++]
            
    #### Now turn off the FEC Counters and Histograms
    for fec_idx in fecs:
       wregBits(fec_class3[fec_idx]+0x63, [1], 0) # unfreeze the counter

    #### Print out Final results and Histograms
    if print_en:
        length_1 = 22
        length = 17

        print("\n  FEC Histogram (%d sec)"%(hist_time)),
        num_fec_en=0
        fec_en=[0]*8
        for fec_idx in fecs:
            if(fec_status_en(fec_idx)):
                fec_en[fec_idx]=1
                num_fec_en+=1
            
        separator='\n +'
        for i in range(length_1):
            separator+='-'
        for i in range(length*num_fec_en):
            separator+='-'*print_en
        separator += '+'   
        
        print separator,
        print '\n |  Parameters          |',
        for fec_idx in fecs:
            if fec_en[fec_idx]:
                fec_type = 'A'   if fec_idx<4 else 'B'
                macro    = fec_idx if fec_idx<4 else fec_idx-4
                print('    FEC_%s[%d]   |'%(fec_type,macro)),
                if print_en==2: print('    FEC_%s[%d]   |'%(fec_type,macro)),

        print separator,           
        print '\n | RX FEC AmLck 0,1,2,3 |',
        for fec_idx in fecs:
            if fec_en[fec_idx]:
                print("       %04d    |"%(result[fec_idx][2])),
                if print_en==2: print("       %04d    |"%(result[fec_idx][2])),
                
        print '\n | RX FEC Uncorr (cw)   |',
        for fec_idx in fecs:
            if fec_en[fec_idx]:
                print(" %10d    |"%(result[fec_idx][3])),
                if print_en==2: print(" %10.1e    |"%(float(result[fec_idx][3])/float(result[fec_idx][5]))),
        print '\n | RX FEC Corr   (cw)   |',
        for fec_idx in fecs:
            if fec_en[fec_idx]:
                print(" %10d    |"%(result[fec_idx][4])),
                if print_en==2: print(" %10.1e    |"%(float(result[fec_idx][4])/float(result[fec_idx][5]))),
        print separator,
        for bin in bins:
            print('\n | RX FEC Bin %2d        |'%(bin )),
            for fec_idx in fecs:
                if fec_en[fec_idx]:
                    print(" %10d    |"%(result[fec_idx][6+bin])),
                    if print_en==2: print(" %10.1e    |"%(float(result[fec_idx][6+bin])/float(result[fec_idx][5]))),
        print separator,

    if not print_en:
        return result 
#############################################################################  
def fec_ber(fec_idx):

    if chip_rev == 1.0:
        print ("*** fec_status_pre_fec_ber() This function is not supported in Baldeagle A0 ***\n")
        return
        
    fec_base_addr = fec_class3[fec_idx]
    cw_size = 5440 if fec_idx < 4 else 5280
    
    wreg(fec_base_addr+0x63,0x8002) # freeze the counters bit[1]=1, turn the clock on for reading counters bit[4]=0
    time.sleep(.010)
    total_errors_hi   = rreg(fec_base_addr+0x7f)
    total_errors_mid  = rreg(fec_base_addr+0x7e)
    total_errors_lo   = rreg(fec_base_addr+0x7d)
 
    total_cw_hi  = rreg(fec_base_addr+0x5f)
    total_cw_mid = rreg(fec_base_addr+0x5e)
    total_cw_lo  = rreg(fec_base_addr+0x5d) 
   
    total_errors = long( (total_errors_hi << 32) + (total_errors_mid<<16) + (total_errors_lo) )
    total_cw     = long( (total_cw_hi     << 32) + (total_cw_mid    <<16) + (total_cw_lo    ) )
    ber = float(total_errors)/float(total_cw)/float(cw_size)
    
    #print ("FEC %d (base: 0x%04X) Pre-FEC BER:"%(fec_idx,fec_base_addr))
    #print ("Errors : %04X_%04X_%04X = %11d bits"%(total_errors_hi,total_errors_mid,total_errors_lo,total_errors))
    #print ("CWs    : %04X_%04X_%04X = %11d CWs"%(total_cw_hi,total_cw_mid,total_cw_lo, total_cw))
    if total_errors == 0 and total_cw != 0:
        ber = 0.0 # print ("BER    : 0")
    elif total_cw == 0:
        ber = -1.0 #print ("BER    : ERROR. CW Count =0 !!!")
    else:
        ber = float(total_errors)/float(total_cw)/float(cw_size)
        #print ("BER    : %2.2e"%(float(total_errors)/float(total_cw)/float(cw_size)))
    
    wreg(fec_base_addr+0x63,0x8000) # unfreeze the counters, bit[1]=0, 
    time.sleep(0.010)
    wreg(fec_base_addr+0x63,0x8001) # clear the counters for next time, bit[0]=1
    time.sleep(0.010)
    wreg(fec_base_addr+0x63,0x8000) # clear the counters for next time, bit[0]=0
    #time.sleep(0.010)
    #wreg(fec_base_addr+0x63,0x8010) # turn the clock off to save power, bit[4]=1
    return total_errors, total_cw, ber
    
# Set any lane in "shallow loop-back mode internally":
#
# A0-RX looped back out to A0-TX
# A1-RX looped back out to A1-TX
# ...
# B6-RX looped back out to B6-TX 
# B7-RX looped back out to B7-TX 
#
####################################################################################################
def sw_config_loopback_mode(lane=None, enable='en'):

    lanes = get_lane_list(lane)

    #### TOP PLL: need to set dvi2 mode
    #wreg(0x9500,0x1a68) ## top pll
    #wreg(0x9600,0x1a68)
    
    #get_lane_mode(lanes) # update current Encoding modes of all lanes for this Slice
    loopback_bit = 1 if enable == 'en' else 0
    
    if loopback_bit==1: # loopback_mode enabled, TX in functional mode  
        wreg(0x9857,0x0000) # Disable FEC
        wreg(0x9858,0x0000) # Disable any lane mapping for Gearbox modes
        wreg(0x9859,0x0000) # Disable BM (BitMux)
        wreg(0x9855,0x0000) # Disable any lane mapping for Gearbox modes
        wreg(0x985f,0x0000) # Disable AN

        wreg(0x98d1,0x8888) # [15:12]=8: B7RX->B7TX, [11:8]=8: B6RX->B6TX, [7:4]=8: B5RX->B5TX, [3:0]=8: B4RX->B4TX
        wreg(0x98d2,0x8888) # [15:12]=8: B3RX->B3TX, [11:8]=8: B2RX->B2TX, [7:4]=8: B1RX->B1TX, [3:0]=8: B0RX->B0TX
        wreg(0x98d3,0x8888) # [15:12]=8: A7RX->A7TX, [11:8]=8: A6RX->A6TX, [7:4]=8: A5RX->A5TX, [3:0]=8: A4RX->A4TX
        wreg(0x98d4,0x8888) # [15:12]=8: A3RX->A3TX, [11:8]=8: A2RX->A2TX, [7:4]=8: A1RX->A1TX, [3:0]=8: A0RX->A0TX
    else:                        # loopback_mode disabled, TX in PRBS mode
        wreg(0x98d1,0x0000) # [15:12]=0: B7_RXPLL->B7TX, [11:8]=0: B6_RXPLL->B6TX, [7:4]=0: B5_RXPLL->B5TX, [3:0]=0: B4_RXPLL->B4TX
        wreg(0x98d2,0x0000) # [15:12]=0: B3_RXPLL->B3TX, [11:8]=0: B2_RXPLL->B2TX, [7:4]=0: B1_RXPLL->B1TX, [3:0]=0: B0_RXPLL->B0TX
        wreg(0x98d3,0x0000) # [15:12]=0: A7_RXPLL->A7TX, [11:8]=0: A6_RXPLL->A6TX, [7:4]=0: A5_RXPLL->A5TX, [3:0]=0: A4_RXPLL->A4TX
        wreg(0x98d4,0x0000) # [15:12]=0: A3_RXPLL->A3TX, [11:8]=0: A2_RXPLL->A2TX, [7:4]=0: A1_RXPLL->A1TX, [3:0]=0: A0_RXPLL->A0TX            

 
    for ln in lanes:      
        get_lane_mode(ln)
        wreg(0x3000+(0x100*ln),0x0000,lane=0) ####disable link training of this lane
        wreg(0xe000+(0x100*ln),0xc000,lane=0) ####disable AutoNeg of this lane  
        
        #### TX rotator and pi enable
        if loopback_bit==0: # Do this first thing if disabling loop-back mode
            wreg([0x0f4,   [0]], loopback_bit, ln) ## TX_PLL refclk Source 0: crystal 1:RX_PLL recovered clock
        wreg([0x0f3,[14,8]], 0x00        , ln) ## [15]->1, TX phase rotator value
        wreg([0x0f3,  [15]], loopback_bit, ln) ## [15]->1, TX phase rotator enable
       #wreg([0x0f3,   [2]], loopback_bit, ln) # 0 or 1 for pam4
        wreg([0x0f3,   [2]],            0, ln) # TRF pol is controlled by register 0x79 or 0x17a


        if loopback_bit==1: # loopback_mode enabled, TX in functional mode
            prbs_mode_select(lane=ln, prbs_mode='functional') # Put TX in Functional Mode
        else:                         # loopback_mode disabled, TX in PRBS mode
            prbs_mode_select(lane=ln, prbs_mode='prbs')    # Put TX in PRBS Mode7
            
        ##### Enable loop-back for this lane
        if ln==0:    #A0
            wreg([0x9811,[1]],loopback_bit,lane=0); wreg([0x9811,[3]],loopback_bit,lane=0);
        elif ln==1:  #A1
            wreg([0x9819,[1]],loopback_bit,lane=0); wreg([0x9819,[3]],loopback_bit,lane=0);
        elif ln==2:  #A2
            wreg([0x9821,[1]],loopback_bit,lane=0); wreg([0x9821,[3]],loopback_bit,lane=0);
        elif ln==3:  #A3
            wreg([0x9829,[1]],loopback_bit,lane=0); wreg([0x9829,[3]],loopback_bit,lane=0);
        elif ln==4:  #A4
            wreg([0x9831,[1]],loopback_bit,lane=0); wreg([0x9831,[3]],loopback_bit,lane=0);
        elif ln==5:  #A5
            wreg([0x9839,[1]],loopback_bit,lane=0); wreg([0x9839,[3]],loopback_bit,lane=0);
        elif ln==6:  #A6
            wreg([0x9841,[1]],loopback_bit,lane=0); wreg([0x9841,[3]],loopback_bit,lane=0);
        elif ln==7:  #A7
            wreg([0x9849,[1]],loopback_bit,lane=0); wreg([0x9849,[3]],loopback_bit,lane=0);
            
        elif ln==8:  #B0
            wreg([0x9811,[0]],loopback_bit,lane=0); wreg([0x9811,[2]],loopback_bit,lane=0);
        elif ln==9:  #B1
            wreg([0x9819,[0]],loopback_bit,lane=0); wreg([0x9819,[2]],loopback_bit,lane=0);
        elif ln==10: #B2
            wreg([0x9821,[0]],loopback_bit,lane=0); wreg([0x9821,[2]],loopback_bit,lane=0);
        elif ln==11: #B3
            wreg([0x9829,[0]],loopback_bit,lane=0); wreg([0x9829,[2]],loopback_bit,lane=0);
        elif ln==12: #B4
            wreg([0x9831,[0]],loopback_bit,lane=0); wreg([0x9831,[2]],loopback_bit,lane=0);
        elif ln==13: #B5
            wreg([0x9839,[0]],loopback_bit,lane=0); wreg([0x9839,[2]],loopback_bit,lane=0);
        elif ln==14: #B6
            wreg([0x9841,[0]],loopback_bit,lane=0); wreg([0x9841,[2]],loopback_bit,lane=0);
        elif ln==15: #B7
            wreg([0x9849,[0]],loopback_bit,lane=0); wreg([0x9849,[2]],loopback_bit,lane=0);
        else:
            print("\nsw_config_loopback_mode: ***> Lane # must be between 0 to 15")
            return
        


        ##### TX peer for this lane's RX is lane_name_list[LanePartnerMap[gSlice][ln]]
        #
        
        if gEncodingMode[gSlice][ln][0] == 'pam4':
            if loopback_bit==1: # PAM4 53G PLL_N
               #wreg(0x1fd,0x113f,ln)  ## perlane RXPLL pll_n
               #wreg([0x1f5,[8]],0,ln) ## perlane RXPLLpll[8]->0: div2 not bypassed
               #wreg(0x079,0x20a4,ln)  ## PAM4 TRF
                wreg(0x079,0x3084,ln)  ## PAM4 TRF
            else:
               #wreg(0x1fd,0x223f,ln)  ## perlane RXPLLpll_n, same as in the init_pam4/init_nrz
               #wreg([0x1f5,[8]],1,ln) ## perlane RXPLLpll[8]->1: div2 bypassed
                wreg(0x079,0x00a4,ln)  ## PAM4 TRF, set to the same value as in the init_pam4/init_nrz

        else: # NRZ mode (TBD, not completed yet)
            if loopback_bit==1: # NRZ 25.78125G PLL_N
               #wreg(0x1fd,0x113f,ln)  ## perlane RXPLL pll_n???
               #wreg([0x1f6,[8]],0,ln) ## perlane RXPLL pll[8]->0: div2 not bypassed
               #wreg(0x17a,0x116C,ln) ##  NRZ TRF
                wreg(0x17a,0x3084,ln) ##  NRZ TRF
            else:
               #wreg(0x1FD,0x213f,ln) # NRZ mode, RX_PLLN= 33*2*195.3125 |  [3:1]  //vcoi=0x7
               #wreg(0x1f6,0x71b0,ln) # NRZ mode, vagcbufdac
                wreg(0x17a,0x00a4,ln) ##  NRZ TRF, set to the same value as in the init_pam4/init_nrz
        
        if loopback_bit==1: # Do this as the last thing when enabling loopback mode
            wreg([0x0f4,   [0]], loopback_bit, ln) ## TX_PLL refclk Source 0: crystal 1:RX_PLL recovered clock
        
####################################################################################################  
# Set A Lanes in retimer mode, one pair of A+B lane at a time
#
# Register         Register   Retimer           Retimer
# Function         Address    Mode              Mode
#                             Direct            Crossed
#                             A0-3 to B0-3      A0-3 to B4-7
#                             A4-7 to B4-7      A4-7 to B0-3
# tx_pi A7-A4      0x98d1     0x7654            0x3210
# tx_pi A3-A0      0x98d2     0x3210            0x7654
# tx_pi B7-B4      0x98d3     0x7654            0x3210
# tx_pi B3-B0      0x98d4     0x3210            0x7654           
# Lane Cross       0x9855     0x0000            0xffff
# AN Lane Cross    0x985f     0x0000            0xff00
#
####################################################################################################
def sw_config_retimer (mode=None, lane=range(8), cross_mode=False):
    print_en=0
    if mode==None: # or all(ln >= 8 for ln in A_lanes): # if any of the A-lanes > 8
        print("\n*** Configure an A+B lane in retimer mode"),
        print("\n*** Options: "),
        print("\n***         mode = 'retimer', 'off' or 'dis'"),
        print("\n***         lane = any of the A lanes, i.e. [0,1,2,3,4,5,6,7]"),
        return
    
    lanes = get_retimer_lane_list(lane, cross_mode)
    A_lanes=[x for x in lanes if x < 8]

    retimer_en = 0 if any(i in mode.upper() for i in ['OFF','DIS','DISABLE']) else 1    
    if retimer_en==1:     ## retimer mode enabled, TX in functional mode
        wreg(0x9857,0x0000) # Disable FEC
        wreg(0x9858,0x0000) # Disable any lane mapping for Gearbox modes
        wreg(0x9859,0x0000) # Disable BitMux
        if cross_mode==False:
            wreg(0x9855,0x0000) # Direct lane mapping for retimer mode
            wreg(0x985f,0x0000) # Direct AN lane mapping for retimer mode
            wreg(0x98d1,0x7654) # lanesA7-A4: A7TX<-B7RX, A6TX<-B6RX, A5TX<-B5RX, A4TX<-B4RX
            wreg(0x98d2,0x3210) # lanesA3-A0: A3TX<-B3RX, A2TX<-B2RX, A1TX<-B1RX, A0TX<-B0RX
            wreg(0x98d3,0x7654) # lanesB7-B4: B7TX<-A7RX, B6TX<-A6RX, B5TX<-A5RX, B4TX<-A4RX
            wreg(0x98d4,0x3210) # lanesB3-B0: B3TX<-A3RX, B2TX<-A2RX, B1TX<-A1RX, B0TX<-A0RX
        else:
            wreg(0x9855,0xFFFF) # Crossed lane mapping for retimer mode
            wreg(0x985f,0xFF00) # Crossed AN lane mapping for retimer mode
            wreg(0x98d1,0x3210) # lanesA7-A4: A7TX<-B3RX, A6TX<-B3RX, A5TX<-B1RX, A4TX<-B0RX
            wreg(0x98d2,0x7654) # lanesA3-A0: A3TX<-B7RX, A2TX<-B6RX, A1TX<-B5RX, A0TX<-B4RX
            wreg(0x98d3,0x3210) # lanesB7-B4: B7TX<-A3RX, B6TX<-A2RX, B5TX<-A1RX, B4TX<-A0RX
            wreg(0x98d4,0x7654) # lanesB3-B0: B3TX<-A7RX, B2TX<-A6RX, B1TX<-A5RX, B0TX<-A4RX
    else:                   # retimer mode disabled, TX in PRBS mode
        wreg(0x9857,0x0000) # Disable FEC
        wreg(0x9858,0x0000) # Disable any lane mapping for Gearbox modes
        wreg(0x9859,0x0000) # Disable BitMux
        wreg(0x9855,0x0000) # Direct lane mapping for retimer mode
        wreg(0x985f,0x0000) # Direct AN lane mapping for retimer mode
        wreg(0x98d1,0x0000) # lanesA7-A4: A7TX<-xxxx, A6TX<-xxxx, A5TX<-xxxx, A4TX<-xxxx
        wreg(0x98d2,0x0000) # lanesA3-A0: A3TX<-xxxx, A2TX<-xxxx, A1TX<-xxxx, A0TX<-xxxx
        wreg(0x98d3,0x0000) # lanesB7-B4: B7TX<-xxxx, B6TX<-xxxx, B5TX<-xxxx, B4TX<-xxxx
        wreg(0x98d4,0x0000) # lanesB3-B0: B3TX<-xxxx, B2TX<-xxxx, B1TX<-xxxx, B0TX<-xxxx            
    
    for ln in lanes:
        get_lane_mode(ln) # PAM4 or NRZ ?        
        wreg(0x3000+(0x100*ln),0x0000,lane=0) ####disable link training of this lane        
        wreg(0xe000+(0x100*ln),0xc000,lane=0) ####disable AutoNeg of this lane
        
        #### TX-PLL and TRF control
        if retimer_en==0: # Turn off TRF the first thing if disabling retimer mode
            wreg([0x0f4,   [0]], retimer_en, ln) ## TX_PLL refclk Source 0: crystal 1:RX_PLL recovered clock
        wreg([0x0f3,[14,8]], 0x00       , ln) ## [15]->1, TX phase rotator value = 0x00
        wreg([0x0f3,  [15]], retimer_en, ln) ## [15]->1, TX phase rotator enable
       #wreg([0x0f3,   [2]], retimer_en, ln) ## TRF pol if needed, 0 or 1
        wreg([0x0f3,   [2]],           0, ln) ## TRF pol is controlled by register 0x79 or 0x17a

        if retimer_en==1: 
            prbs_mode_select(lane=ln, prbs_mode='functional') # Put TX in Functional Mode
        else:                         
            prbs_mode_select(lane=ln, prbs_mode='prbs')    # Put TX in PRBS Mode7
                    
        ##### TX peer for this lane's RX is lane_name_list[LanePartnerMap[gSlice][ln]] #
        
        ###### PAM4 mode
        if gEncodingMode[gSlice][ln][0] == 'pam4':
            if retimer_en==1: # PAM4 53G
                wreg(0x079,0x5044,ln)  ## PAM4 TRF
            else:
                wreg(0x079,0x00a4,ln)  ## PAM4 TRF, set to the same value as in the init_pam4/init_nrz
        ###### NRZ mode
        else: 
            if retimer_en==1: # NRZ 25G
                wreg(0x17a,0x1044,ln) ##  NRZ TRF
            else:
                wreg(0x17a,0x00a4,ln) ##  NRZ TRF, set to the same value as in the init_pam4/init_nrz
 
        if retimer_en==1: # Do this as the last thing when enabling retimer mode
            wreg([0x0f4,   [0]], retimer_en, ln) ## TX_PLL refclk Source 0: crystal 1:RX_PLL recovered clock
            
    ######## Done with enabling or disabling the retimers #############
    for ln in A_lanes:
        if cross_mode==False:
            b_ln = ln
            tag = '<--->'
            tag2 = 'Direct Mode'
        elif ln<4:
            b_ln = ln+4
            tag = '<-X->'
            tag2 = 'Crossed Mode'
        elif ln>=4:
            b_ln = ln-4
            tag = '<-X->'          
            tag2 = 'Crossed Mode'
        if retimer_en==1:
            if print_en: print("\n...SW Retimer: Enabled Retimer in %s between Lanes A%d %s B%d"%(tag2,ln,tag,b_ln)),
        else: # retimer disabled
            if print_en: print("\n...SW Retimer: Disabled Retimer on Lane A%d"%(ln)),

####################################################################################################  
# FW to Configure lanes in Bitmux A1:B2 NRZ-NRZ mode
#
# A-side NRZ 2x20G  to  B-side: NRZ 4x10G
# A-lanes are running exactly at 2X the data rate of the B-lanes
# Must initialize and optimize the lanes in at proper data rate before calling this routine
#
# Options: A_lanes=[0,1]
#          A_lanes=[2,3]
#          A_lanes=[4,5]
#          A_lanes=[0,1,2,3]
#          A_lanes=[0,1,4,5]
####################################################################################################
def fw_config_bitmux_20G(A_lanes=[0,1],print_en=0):

    if not fw_loaded(print_en=0):
        print("\n...FW Bitmux 20G: FW not loaded. BITMUX Not Configured!"),
        return
    
    
    
    # For 2x20G to 4x10G Bitmux mode, 3 options supported for A-Lane groups 
    group1_bitmux=[0,1] # A_lanes group 1 -> [A0,A1] <-> [B0,B1,B2,B3]
    group2_bitmux=[2,3] # A_lanes group 2 -> [A2,A3] <-> [B4,B5,B6,B7]
    group3_bitmux=[4,5] # A_lanes group 3 -> [A4,A5] <-> [B4,B5,B6,B7]
    
    #Determine the corresponding B-Lanes for each group of A-Lanes
    group1_selected=0
    group2_selected=0
    B_lanes=[]
    if all(elem in A_lanes for elem in group1_bitmux):  # If A_lanes contains [0,1]
        B_lanes +=[8,9,10,11]
        group1_selected=1
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [2,3]
        B_lanes+=[12,13,14,15]
        group2_selected=1
    elif all(elem in A_lanes for elem in group3_bitmux): # If A_lanes contains [4,5]
        B_lanes+=[12,13,14,15]
        group2_selected=1
    
    if group1_selected==0 and group2_selected==0:
        print("\n*** Bitmux Setup: Invalid Target A-Lanes specified!"),
        print("\n*** Options: A_lanes=[0,1]"),
        print("\n***          A_lanes=[2,3]"),
        print("\n***          A_lanes=[4,5]"),
        print("\n***          A_lanes=[0,1,2,3]"),
        print("\n***          A_lanes=[0,1,4,5]"),
        return
    
    lanes = sorted(list(set(A_lanes + B_lanes)))
    #prbs_mode_select(lane=lanes, prbs_mode='functional')
            
    ######## First, do all the destroys #############
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [0,1]
        fw_config_cmd(config_cmd=0x9040,config_detail=0x0000) # 0x9040 = First, FW destroy any instances of these lanes being already used      
        fw_config_cmd(config_cmd=0x9041,config_detail=0x0000) # 0x9041 = First, FW destroy any instances of these lanes being already used
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [2,3]
        fw_config_cmd(config_cmd=0x9042,config_detail=0x0000) # 0x9041 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9043,config_detail=0x0000) # 0x9042 = First, FW destroy any instances of these lanes being already used     
        fw_config_cmd(config_cmd=0x9044,config_detail=0x0000) # 0x9043 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9045,config_detail=0x0000) # 0x9044 = First, FW destroy any instances of these lanes being already used
    elif all(elem in A_lanes for elem in group3_bitmux): # If A_lanes contains [4,5]
        fw_config_cmd(config_cmd=0x9042,config_detail=0x0000) # 0x9041 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9043,config_detail=0x0000) # 0x9042 = First, FW destroy any instances of these lanes being already used     
        fw_config_cmd(config_cmd=0x9044,config_detail=0x0000) # 0x9043 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9045,config_detail=0x0000) # 0x9044 = First, FW destroy any instances of these lanes being already used


    ######## Next, do all the activates #############
    
    ######## Bitmux A2/A3 to B4/B5/B6/B7    
    if all(elem in A_lanes for elem in group1_bitmux):  # If A_lanes contains [0,1]
        if print_en: print("\n...FW BitMux 20G : Setting Up Lane A0/A1 to B0/B1/B2/B3..."),
        fw_config_cmd(config_cmd=0x8040,config_detail=0x0021) # 0x8040 = FW to Activate Bitmux A1B2 NRZ for Lane A0, 0x0021 = FW Bitmux A1B2 NRZ command        
        fw_config_cmd(config_cmd=0x8041,config_detail=0x0021) # 0x8041 = FW to Activate Bitmux A1B2 NRZ for Lane A1, 0x0021 = FW Bitmux A1B2 NRZ command       

    ######## Bitmux A2/A3 to B4/B5/B6/B7
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [2,3]
        if print_en: print("\n...FW BitMux 20G : Setting Up Lane A2/A3 to B4/B5/B6/B7..."),
        fw_config_cmd(config_cmd=0x8042,config_detail=0x0021) # 0x8042 = FW to Activate Bitmux A1B2 NRZ for Lane A2, 0x0021 = FW Bitmux A1B2 NRZ command       
        fw_config_cmd(config_cmd=0x8043,config_detail=0x0021) # 0x8043 = FW to Activate Bitmux A1B2 NRZ for Lane A3, 0x0021 = FW Bitmux A1B2 NRZ command       

    ######## Bitmux A4/A5 to B4/B5/B6/B7    
    elif all(elem in A_lanes for elem in group3_bitmux): # If A_lanes contains [4,5]
        if print_en: print("\n...FW BitMux 20G : Setting Up Lane A4/A5 to B4/B5/B6/B7..."),
        fw_config_cmd(config_cmd=0x8044,config_detail=0x0021) # 0x8044 = FW to Activate Bitmux A1B2 NRZ for Lane A4, 0x0021 = FW Bitmux A1B2 NRZ command
        fw_config_cmd(config_cmd=0x8045,config_detail=0x0021) # 0x8045 = FW to Activate Bitmux A1B2 NRZ for Lane A5, 0x0021 = FW Bitmux A1B2 NRZ command

    #prbs_mode_select(lane=lanes, prbs_mode='functional')
    #fw_adapt_wait(max_wait=20, lane=A_lanes+B_lanes, print_en=1)

    return A_lanes, B_lanes
####################################################################################################  
# FW to Configure lanes in Bitmux A1:B2 PAM4-NRZ mode
#
# A-side NRZ 2x53G  to  B-side: NRZ 4x26G
# A-lanes are running exactly at 2X the data rate of the B-lanes
# Must initialize and optimize the lanes in at proper data rate before calling this routine
#
# Options: A_lanes=[0,1]
#          A_lanes=[2,3]
#          A_lanes=[4,5]
#          A_lanes=[0,1,2,3]
#          A_lanes=[0,1,4,5]
####################################################################################################
def fw_config_bitmux_53G(A_lanes=[0,1],print_en=0):

    if not fw_loaded(print_en=0):
        print("\n...FW Bitmux 53G: FW not loaded. BITMUX Not Configured!"),
        return

    # For 2x20G to 4x10G Bitmux mode, 3 options supported for A-Lane groups 
    group1_bitmux=[0,1] # A_lanes group 1 -> [A0,A1] <-> [B0,B1,B2,B3]
    group2_bitmux=[2,3] # A_lanes group 2 -> [A2,A3] <-> [B4,B5,B6,B7]
    group3_bitmux=[4,5] # A_lanes group 3 -> [A4,A5] <-> [B4,B5,B6,B7]
    
    #Determine the corresponding B-Lanes for each group of A-Lanes
    group1_selected=0
    group2_selected=0
    B_lanes=[]
    if all(elem in A_lanes for elem in group1_bitmux):  # If A_lanes contains [0,1]
        B_lanes +=[8,9,10,11]
        group1_selected=1
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [2,3]
        B_lanes+=[12,13,14,15]
        group2_selected=1
    elif all(elem in A_lanes for elem in group3_bitmux): # If A_lanes contains [4,5]
        B_lanes+=[12,13,14,15]
        group2_selected=1
    
    if group1_selected==0 and group2_selected==0:
        print("\n*** Bitmux Setup: Invalid Target A-Lanes specified!"),
        print("\n*** Options: A_lanes=[0,1]"),
        print("\n***          A_lanes=[2,3]"),
        print("\n***          A_lanes=[4,5]"),
        print("\n***          A_lanes=[0,1,2,3]"),
        print("\n***          A_lanes=[0,1,4,5]"),
        return
    
    lanes = sorted(list(set(A_lanes + B_lanes)))
    #prbs_mode_select(lane=lanes, prbs_mode='functional')

        
    ######## First, do all the destroys #############
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [0,1]
        fw_config_cmd(config_cmd=0x9050,config_detail=0x0000) # 0x9050 = First, FW destroy any instances of these lanes being already used      
        fw_config_cmd(config_cmd=0x9051,config_detail=0x0000) # 0x9051 = First, FW destroy any instances of these lanes being already used
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [2,3]
        fw_config_cmd(config_cmd=0x9052,config_detail=0x0000) # 0x9052 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9053,config_detail=0x0000) # 0x9053 = First, FW destroy any instances of these lanes being already used     
        fw_config_cmd(config_cmd=0x9054,config_detail=0x0000) # 0x9054 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9055,config_detail=0x0000) # 0x9055 = First, FW destroy any instances of these lanes being already used
    elif all(elem in A_lanes for elem in group3_bitmux): # If A_lanes contains [4,5]
        fw_config_cmd(config_cmd=0x9052,config_detail=0x0000) # 0x9052 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9053,config_detail=0x0000) # 0x9053 = First, FW destroy any instances of these lanes being already used     
        fw_config_cmd(config_cmd=0x9054,config_detail=0x0000) # 0x9054 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9055,config_detail=0x0000) # 0x9055 = First, FW destroy any instances of these lanes being already used


    
    ######## Next, do all the activates #############
    
    ######## Bitmux A2/A3 to B4/B5/B6/B7    
    if all(elem in A_lanes for elem in group1_bitmux):  # If A_lanes contains [0,1]
        if print_en: print("\n...FW BitMux 53G : Setting Up Lane A0/A1 to B0/B1/B2/B3..."),
        fw_config_cmd(config_cmd=0x8050,config_detail=0x0064) # 0x8050 = FW to Activate Bitmux 53G for Lane A0, 0x0064 = FW Bitmux 53G command        
        fw_config_cmd(config_cmd=0x8051,config_detail=0x0064) # 0x8051 = FW to Activate Bitmux 53G for Lane A1, 0x0064 = FW Bitmux 53G command       

                                                                                                   
    ######## Bitmux A2/A3 to B4/B5/B6/B7                                                          
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [2,3]               
        if print_en: print("\n...FW BitMux 53G : Setting Up Lane A2/A3 to B4/B5/B6/B7..."),       
        fw_config_cmd(config_cmd=0x8052,config_detail=0x0064) # 0x8052 = FW to Activate Bitmux 53G for Lane A2, 0x0064 = FW Bitmux 53G command       
        fw_config_cmd(config_cmd=0x8053,config_detail=0x0064) # 0x8053 = FW to Activate Bitmux 53G for Lane A3, 0x0064 = FW Bitmux 53G command       
    
    
    ######## Bitmux A4/A5 to B4/B5/B6/B7                                                                      
    elif all(elem in A_lanes for elem in group3_bitmux): # If A_lanes contains [4,5]                          
        if print_en: print("\n...FW BitMux 53G : Setting Up Lane A4/A5 to B4/B5/B6/B7..."),                   
        fw_config_cmd(config_cmd=0x8054,config_detail=0x0064) # 0x8054 = FW to Activate Bitmux 53G for Lane A4, 0x0064 = FW Bitmux 53G command
        fw_config_cmd(config_cmd=0x8055,config_detail=0x0064) # 0x8055 = FW to Activate Bitmux 53G for Lane A5, 0x0064 = FW Bitmux 53G command


    #fw_adapt_wait(max_wait=20, lane=A_lanes+B_lanes, print_en=1)


    return A_lanes, B_lanes
    
def fw_config_bitmux_51G(A_lanes=[0,1],print_en=0):

    if not fw_loaded(print_en=0):
        print("\n...FW Bitmux 53G: FW not loaded. BITMUX Not Configured!"),
        return

    # For 2x20G to 4x10G Bitmux mode, 3 options supported for A-Lane groups 
    group1_bitmux=[0,1] # A_lanes group 1 -> [A0,A1] <-> [B0,B1,B2,B3]
    group2_bitmux=[2,3] # A_lanes group 2 -> [A2,A3] <-> [B4,B5,B6,B7]
    group3_bitmux=[4,5] # A_lanes group 3 -> [A4,A5] <-> [B4,B5,B6,B7]
    
    #Determine the corresponding B-Lanes for each group of A-Lanes
    group1_selected=0
    group2_selected=0
    B_lanes=[]
    if all(elem in A_lanes for elem in group1_bitmux):  # If A_lanes contains [0,1]
        B_lanes +=[8,9,10,11]
        group1_selected=1
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [2,3]
        B_lanes+=[12,13,14,15]
        group2_selected=1
    elif all(elem in A_lanes for elem in group3_bitmux): # If A_lanes contains [4,5]
        B_lanes+=[12,13,14,15]
        group2_selected=1
    
    if group1_selected==0 and group2_selected==0:
        print("\n*** Bitmux Setup: Invalid Target A-Lanes specified!"),
        print("\n*** Options: A_lanes=[0,1]"),
        print("\n***          A_lanes=[2,3]"),
        print("\n***          A_lanes=[4,5]"),
        print("\n***          A_lanes=[0,1,2,3]"),
        print("\n***          A_lanes=[0,1,4,5]"),
        return
    
    lanes = sorted(list(set(A_lanes + B_lanes)))
    #prbs_mode_select(lane=lanes, prbs_mode='functional')

        
    ######## First, do all the destroys #############
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [0,1]
        fw_config_cmd(config_cmd=0x9060,config_detail=0x0000) # 0x9050 = First, FW destroy any instances of these lanes being already used      
        fw_config_cmd(config_cmd=0x9061,config_detail=0x0000) # 0x9051 = First, FW destroy any instances of these lanes being already used
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [2,3]
        fw_config_cmd(config_cmd=0x9062,config_detail=0x0000) # 0x9052 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9063,config_detail=0x0000) # 0x9053 = First, FW destroy any instances of these lanes being already used     
        fw_config_cmd(config_cmd=0x9064,config_detail=0x0000) # 0x9054 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9065,config_detail=0x0000) # 0x9055 = First, FW destroy any instances of these lanes being already used
    elif all(elem in A_lanes for elem in group3_bitmux): # If A_lanes contains [4,5]
        fw_config_cmd(config_cmd=0x9062,config_detail=0x0000) # 0x9052 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9063,config_detail=0x0000) # 0x9053 = First, FW destroy any instances of these lanes being already used     
        fw_config_cmd(config_cmd=0x9064,config_detail=0x0000) # 0x9054 = First, FW destroy any instances of these lanes being already used
        fw_config_cmd(config_cmd=0x9065,config_detail=0x0000) # 0x9055 = First, FW destroy any instances of these lanes being already used


    
    ######## Next, do all the activates #############
    
    ######## Bitmux A2/A3 to B4/B5/B6/B7    
    if all(elem in A_lanes for elem in group1_bitmux):  # If A_lanes contains [0,1]
        if print_en: print("\n...FW BitMux 53G : Setting Up Lane A0/A1 to B0/B1/B2/B3..."),
        fw_config_cmd(config_cmd=0x8060,config_detail=0x0083) # 0x8050 = FW to Activate Bitmux 53G for Lane A0, 0x0064 = FW Bitmux 53G command        
        fw_config_cmd(config_cmd=0x8061,config_detail=0x0083) # 0x8051 = FW to Activate Bitmux 53G for Lane A1, 0x0064 = FW Bitmux 53G command       

                                                                                                   
    ######## Bitmux A2/A3 to B4/B5/B6/B7                                                          
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [2,3]               
        if print_en: print("\n...FW BitMux 53G : Setting Up Lane A2/A3 to B4/B5/B6/B7..."),       
        fw_config_cmd(config_cmd=0x8062,config_detail=0x0083) # 0x8052 = FW to Activate Bitmux 53G for Lane A2, 0x0064 = FW Bitmux 53G command       
        fw_config_cmd(config_cmd=0x8063,config_detail=0x0083) # 0x8053 = FW to Activate Bitmux 53G for Lane A3, 0x0064 = FW Bitmux 53G command       
    
    
    ######## Bitmux A4/A5 to B4/B5/B6/B7                                                                      
    elif all(elem in A_lanes for elem in group3_bitmux): # If A_lanes contains [4,5]                          
        if print_en: print("\n...FW BitMux 53G : Setting Up Lane A4/A5 to B4/B5/B6/B7..."),                   
        fw_config_cmd(config_cmd=0x8064,config_detail=0x0083) # 0x8054 = FW to Activate Bitmux 53G for Lane A4, 0x0064 = FW Bitmux 53G command
        fw_config_cmd(config_cmd=0x8065,config_detail=0x0083) # 0x8055 = FW to Activate Bitmux 53G for Lane A5, 0x0064 = FW Bitmux 53G command


    #fw_adapt_wait(max_wait=20, lane=A_lanes+B_lanes, print_en=1)


    return A_lanes, B_lanes
####################################################################################################  
# SW to Configure lanes in Bitmux A1:B2 NRZ-NRZ mode
#
# A-side NRZ 2x20G  to  B-side: NRZ 4x10G
# A-lanes are running exactly at 2X the data rate of the B-lanes
# Must initialize and optimize the lanes in at proper data rate before calling this routine
#
# Options: A_lanes=[0,1]
#          A_lanes=[2,3]
#          A_lanes=[4,5]
#          A_lanes=[0,1,2,3]
#          A_lanes=[0,1,4,5]
####################################################################################################
def sw_config_bitmux_20G(A_lanes=[0,1],print_en=0):
    
    print_en=1
        
    # For 2x20G to 4x10G Bitmux mode, 3 options supported for A-Lane groups 
    group1_bitmux=[0,1] # A_lanes group 1 -> [A0,A1] <-> [B0,B1,B2,B3]
    group2_bitmux=[2,3] # A_lanes group 2 -> [A2,A3] <-> [B4,B5,B6,B7]
    group3_bitmux=[4,5] # A_lanes group 3 -> [A4,A5] <-> [B4,B5,B6,B7]
    
    #Determine the corresponding B-Lanes for each group of A-Lanes
    group1_selected=0
    group2_selected=0
    B_lanes=[]
    if all(elem in A_lanes for elem in group1_bitmux):  # If A_lanes contains [0,1]
        B_lanes +=[8,9,10,11]
        group1_selected=1

    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [2,3]
        B_lanes+=[12,13,14,15]
        group2_selected=1
    elif all(elem in A_lanes for elem in group3_bitmux): # If A_lanes contains [4,5]
        B_lanes+=[12,13,14,15]
        group2_selected=1
    
    if group1_selected==0 and group2_selected==0:
        print("\n*** Bitmux Setup: Invalid Target A-Lanes specified!"),
        print("\n*** Options: A_lanes=[0,1]"),
        print("\n***          A_lanes=[2,3]"),
        print("\n***          A_lanes=[4,5]"),
        print("\n***          A_lanes=[0,1,2,3]"),
        print("\n***          A_lanes=[0,1,4,5]"),
        return
    
    lanes = sorted(list(set(A_lanes + B_lanes)))
    prbs_mode_select(lane=lanes, prbs_mode='functional')
    

    for ln in A_lanes: ################# A lanes general setup for bitmux mode
        wreg(0x3000+0x100*ln,0x0000,0) # LT dis
        wreg([0x079,[12,0]],0x1044,ln) # PAM4 TRF not used in bitmux nrz-nrz
        wreg([0x17a,[12,0]],0x1044,ln) # NRZ TRF for bitmux mode
        wreg([0x0f4,[0]],        1,ln) # TXPLL use RX Recovered 
        wreg([0x0f3,[2]],        0,ln) # TX phase rotator flip
        wreg([0x0f3,[15]],       1,ln) # TX phase rotator en
        wreg([0x0FB,[0]],        0,ln) # TX FEC Clock dis
        wreg([0x0F1,[3]],        0,ln) # RX FEC Clock dis
        
    for ln in B_lanes: ################# B lanes general setup for bitmux mode
        wreg(0x3000+0x100*ln,0x0000,0) # LT dis
        wreg([0x079,[12,0]],0x1044,ln) # PAM4 TRF not used in bitmux nrz-nrz
        wreg([0x17a,[12,0]],0x1044,ln) # NRZ TRF for bitmux mode
        wreg([0x0f4,[0]],        1,ln) # TXPLL use RX Recovered 
        wreg([0x0f3,[2]],        0,ln) # TX phase rotator flip
        wreg([0x0f3,[15]],       1,ln) # TX phase rotator en
        wreg([0x0FB,[0]],        0,ln) # TX FEC Clock dis     
        wreg([0x0F1,[3]],        0,ln) # RX FEC Clock dis
       
    if all(elem in A_lanes for elem in group1_bitmux):  # If A_lanes contains [0,1]
        if print_en: print("\nBit-Mux A1:B2 NRZ-NRZ : Setting Up Lanes A0/A1 to B0/B1/B2/B3..."),
        wreg([0x98d2,[ 7, 4]], 2) # A1TX<-B2RX
        wreg([0x98d2,[ 3, 0]], 0) # A0TX<-B0RX
        wreg([0x98d4,[15,12]], 1) # B3TX<-A1RX 
        wreg([0x98d4,[11, 8]], 1) # B2TX<-A1RX 
        wreg([0x98d4,[ 7, 4]], 0) # B1TX<-A0RX 
        wreg([0x98d4,[ 3, 0]], 0) # B0TX<-A0RX 
        wreg([0x9859,  [1,0]], 0) # Bitmux 21_1 and 21_0 disabled
        wreg([0x9859,  [5,4]], 3) # Bitmux 12_1 and 12_0 enabled
        wreg([0x9859,  [9,8]], 3) # Bitmux mode 0 =NRZ and mode 1 = NRZ
       #wreg([0x9858,  [1,0]], 0) # Line mapping: laneA0 always to lanesB0/B1. No need to program line mapping 
       #wreg([0x9858,  [3,2]], 0) # Line mapping: laneA1 always to lanesB2/B3. No need to program line mapping 
        wreg([0x9857,    [0]], 0) # FEC_A0 Disabled  (for 100G-2 lanes A0/A1 to B0/B1/B2/B3)
        wreg([0x9857,    [4]], 0) # FEC_B0 Disabled  (for 100G-2 lanes A0/A1 to B0/B1/B2/B3)
        FEC_A_IDX = 0 # FEC_A0
        FEC_B_IDX = 4 # FEC_B0
        #fec_reset(fec_list=[FEC_A_IDX, FEC_B_IDX])
        
    if all(elem in A_lanes for elem in group2_bitmux):  # If A_lanes contains [2,3]
        if print_en: print("\nBit-Mux A1:B2 NRZ-NRZ : Setting Up Lanes A2/A3 to B4/B5/B6/B7..."),
        wreg([0x98d2,[15,12]], 6) # A3TX<-B6RX
        wreg([0x98d2,[11, 8]], 4) # A2TX<-B4RX
        wreg([0x98d3,[15,12]], 3) # B7TX<-A3RX 
        wreg([0x98d3,[11, 8]], 3) # B6TX<-A3RX 
        wreg([0x98d3,[ 7, 4]], 2) # B5TX<-A2RX 
        wreg([0x98d3,[ 3, 0]], 2) # B4TX<-A2RX 
        wreg([0x9859,  [3,2]], 0) # Bitmux 21_3 and 21_2 disabled
        wreg([0x9859,  [7,6]], 3) # Bitmux 12_3 and 12_2 enabled
        wreg([0x9859,[11,10]], 3) # Bitmux mode 3 = NRZ and mode 2 = NRZ
        wreg([0x9858,  [5,4]], 2) # Line mapping: lane A2 to lanes B4/B5 
        wreg([0x9858,  [7,6]], 2) # Line mapping: lane A3 to lanes B6/B7
        wreg([0x9857,    [2]], 0) # FEC_A2 Disabled  (for 100G-2 lanes A2/A3 to B4/B5/B6/B7)
        wreg([0x9857,    [6]], 0) # FEC_B2 Disabled  (for 100G-2 lanes A2/A3 to B4/B5/B6/B7)
        FEC_A_IDX = 2 # FEC_A2
        FEC_B_IDX = 6 # FEC_B2
        #fec_reset(fec_list=[FEC_A_IDX, FEC_B_IDX])
        
    elif all(elem in A_lanes for elem in group3_bitmux): # If A_lanes contains [4,5]
        if print_en: print("\nBit-Mux A1:B2 NRZ-NRZ : Setting Up Lanes A4/A5 to B4/B5/B6/B7..."),
        wreg([0x98d1,[ 7, 4]], 6) # A5TX<-B6RX
        wreg([0x98d1,[ 3, 0]], 4) # A4TX<-B4RX
        wreg([0x98d3,[15,12]], 5) # B7TX<-A5RX 
        wreg([0x98d3,[11, 8]], 5) # B6TX<-A5RX 
        wreg([0x98d3,[ 7, 4]], 4) # B5TX<-A4RX 
        wreg([0x98d3,[ 3, 0]], 4) # B4TX<-A4RX 
        wreg([0x9859,  [3,2]], 0) # Bitmux 21_3 and 21_2 disabled
        wreg([0x9859,  [7,6]], 3) # Bitmux 12_3 and 12_2 enabled
        wreg([0x9859,[11,10]], 3) # Bitmux mode 3 = NRZ and mode 2 = NRZ
        wreg([0x9858,  [5,4]], 0) # Line mapping: lane A4 to lanes B4/B5 
        wreg([0x9858,  [7,6]], 0) # Line mapping: lane A5 to lanes B6/B7
        wreg([0x9857,    [2]], 0) # FEC_A2 Disabled  (for 100G-2 lanes A4/A5 to B4/B5/B6/B7)
        wreg([0x9857,    [6]], 0) # FEC_B2 Disabled  (for 100G-2 lanes A4/A5 to B4/B5/B6/B7)
        FEC_A_IDX = 2 # FEC_A2
        FEC_B_IDX = 6 # FEC_B2
        #fec_reset(fec_list=[FEC_A_IDX, FEC_B_IDX])

    return A_lanes, B_lanes

####################################################################################################
# Instruct the FW to configure an A+B lane in retimer mode
# 
# Options:
#     mode    = 'pam4', 'nrz'/'nrz25', 'nrz10', 'nrz20'"),
#     A_lanes = any of the A lanes, i.e. [0,1,2,3,4,5,6,7]"),
#
##############################################################################
def fw_config_retimer (mode=None, lane=range(8), cross_mode=False,print_en=0):

    if not fw_loaded(print_en=0):
        print("\n*** No FW Loaded. Skipping fw_config_retimer().\n")
        return
    
    retimer_en=1;
    LT_en=1 if 'LT' in mode.upper() else 0

    if mode==None: # or all(ln >= 8 for ln in A_lanes): # if any of the A-lanes > 8
        print("\n*** Instruct the FW to configure an A+B lane in retimer mode"),
        print("\n*** Options: "),
        print("\n***         mode = 'pam4', 'pam4-LT', 'nrz'/'nrz25'/'nrz-LT', 'nrz10', 'nrz20'"),
        print("\n***         lane = any of the A lanes, i.e. [0,1,2,3,4,5,6,7]"),
        return
    
    lanes = get_lane_list(lane)
    A_lanes=[x for x in lanes if x < 8]
    
    speed_str_list =['10','20','25','26','51','53'] # speed as part of mode argument
    speed_code_list=[0x11,0x22,0x33,0x44,0x55,0x66] # speed codes to be written to 0x9807[7:0]
    
    if 'NRZ' in mode.upper():
        destroy_cmd = 0x9000 if cross_mode==False else 0x9020  # command to destroy a NRZ lane
        config_cmd  = 0x8000 if cross_mode==False else 0x8020  # command to activate lane in NRZ      
        speed_code_cmd = 0x33                                  # default NRZ speed is 25G
        for i in range(len(speed_str_list)): # if NRZ speed is specified as part of the mode, take it
            if speed_str_list[i] in mode: speed_code_cmd = speed_code_list[i]
        if LT_en:
            speed_code_cmd  += 0x200
        
    elif 'PAM4' in mode.upper():
        destroy_cmd = 0x9010 if cross_mode==False else 0x9030  # command to destroy a PAM4 lane
        config_cmd  = 0x8010 if cross_mode==False else 0x8030  # command to activate lane in PAM4
        speed_code_cmd = 0x66                                  # default PAM4 speed is 53G
        for i in range(len(speed_str_list)): # if PAM4 speed is specified as part of the mode, take it
            if speed_str_list[i] in mode: speed_code_cmd = speed_code_list[i]
        if LT_en:
            speed_code_cmd  += 0x200
            
    elif any(i in mode.upper() for i in ['OFF','DIS','DISABLE']): 
        destroy_cmd = 0x9010 if cross_mode==False else 0x9030  # command to destroy a retimer lane
        retimer_en=0
    else:
        for ln in A_lanes:
            print("\n*** fw_config_retimer_mode: selected mode '%s' for lane A%d is invalid" %(mode.upper(), ln)),
        return
        
    ######## First, do all the destroys #############
    for ln in A_lanes:
        result = fw_config_cmd (config_cmd=destroy_cmd+ln,config_detail=0x0000) 
        if (result!=c.fw_config_lane_status): # fw_config_lane_status=0x800
            print("\n***Lane %s: FW could not free up lane before reconfiguring it as retimer. (Error Code 0x%04x)" %(lane_name_list[ln],result)),
    #for ln in A_lanes:
    #    wreg([0xa0,[13]],0,ln)    # Make sure TX output is not squelched on A-side
    #    if cross_mode==False:
    #        wreg([0xa0,[13]],0,ln+8)  # Make sure TX output is not squelched on B-side
    #    else:
    #        wreg([0xa0,[13]],0,ln+12) # Make sure TX output is not squelched on B-side
        
    ######## Next, do all the activates if retimer_en=1 #############
    for ln in A_lanes:
        if cross_mode==False:
            b_ln = ln
            tag = '<--->'
            tag2 = 'Direct non-LT Mode' if LT_en==0 else 'Direct LT Mode'
        elif ln<4:
            b_ln = ln+4
            tag = '<-X->'
            tag2 = 'Crossed non-LT Mode' if LT_en==0 else 'Crossed LT Mode'
        elif ln>=4:
            b_ln = ln-4
            tag = '<-X->'          
            tag2 = 'Crossed non-LT Mode' if LT_en==0 else 'Crossed LT Mode'
        if retimer_en==1:
            if print_en: print("\n...FW Retimer: Enabled Retimer in %s between Lanes A%d %s B%d"%(tag2,ln,tag,b_ln)),
            result = fw_config_cmd (config_cmd=config_cmd+ln,config_detail=speed_code_cmd) 
            if (result!=c.fw_config_lane_status): # fw_config_lane_status=0x800
                print("\n***Slice %d Lane %s FW_CONFIG_RETIMER Failed. (SpeedCode 0x9807=0x%04X, ActiveCode 0x9806=0x%04X, ExpectedStatus:0x%0X, ActualStatus=0x%04x)" %(gSlice,lane_name_list[ln],speed_code_cmd, config_cmd+ln,c.fw_config_lane_status,result)),
        else: # retimer disabled
            if print_en: print("\n...FW Retimer: Disabled Retimer on Lane A%d"%(ln)),
    #for ln in A_lanes:
    #    wreg([0xa0,[13]],0,ln)    # Make sure TX output is not squelched on A-side
    #    if cross_mode==False:
    #        wreg([0xa0,[13]],0,ln+8)  # Make sure TX output is not squelched on B-side
    #    else:
    #        wreg([0xa0,[13]],0,ln+12) # Make sure TX output is not squelched on B-side

####################################################################################################  
# Set PAM4-NRZ Lanes in Gear Box mode, A-side PAM4, B-side: NRZ
#
#
####################################################################################################
def sw_config_gearbox_100G(A_lanes=[0,1], fec_b_byp=False):
    
    print_en=1
    
    #fec_reset() # reset all 8 FECs and clear their Align Markers
    
    # For 100G-2 Gearbox mode, 3 options supported for A-Lane groups 
    group0_100G=[0,1] # A_lanes group 1 -> [A0,A1] <-> [ 8, 9,10,11]
    group1_100G=[2,3] # A_lanes group 2 -> [A2,A3] <-> [12,13,14,15]
    group2_100G=[4,5] # A_lanes group 3 -> [A4,A5] <-> [12,13,14,15]
    
    #Determine the corresponding B-Lanes for each group of A-Lanes
    B_lanes=[]
    if all(elem in A_lanes for elem in group0_100G):  # If A_lanes contains [0,1]
        B_lanes+=[8,9,10,11]
    if all(elem in A_lanes for elem in group1_100G):  # If A_lanes contains [2,3]
        B_lanes+=[12,13,14,15]
    elif all(elem in A_lanes for elem in group2_100G): # If A_lanes contains [4,5]
        B_lanes+=[12,13,14,15]
    #else:
    #    print("\n*** 100G-2 Gearbox Setup: Invalid Target A-Lanes specified!\n")
    #    return
    
    lanes = sorted(list(set(A_lanes + B_lanes)))
    prbs_mode_select(lane=lanes, prbs_mode='functional')
    
    for ln in A_lanes: ################# PAM4 lanes general setup for gearbox mode
        wreg(0x3000+0x100*ln,0x0000,0) # LT dis
        wreg([0x079,[12,0]],0x1454,ln) # PAM4 TRF for 50G-to-25G
        wreg([0x0f4,[0]],        1,ln) # TXPLL use RX Recovered 
        wreg([0x0f3,[2]],        0,ln) # TX phase rotator flip
        wreg([0x0f3,[15]],       1,ln) # TX phase rotator en
        wreg([0x0FB,[0]],        1,ln) # TX FEC Clock en
        wreg([0x0F1,[3]],        1,ln) # RX FEC Clock en 
        
    for ln in B_lanes: ################# NRZ lanes general setup for gearbox mode
        wreg(0x3000+0x100*ln,0x0000,0) # LT dis
        wreg([0x17a,[12,0]],0x114C,ln) # NRZ TRF for 25G-to-50G
        wreg([0x0f4,[0]],        1,ln) # TXPLL use RX Recovered 
        wreg([0x0f3,[2]],        0,ln) # TX phase rotator flip
        wreg([0x0f3,[15]],       1,ln) # TX phase rotator en
        wreg([0x0FB,[0]],        1,ln) # TX FEC Clock en        
        wreg([0x0F1,[3]],        1,ln) # RX FEC Clock en
       
    ###top_tx_pi_sel###
    # wreg(0x98d1, 0x0000)    #laneA7-A4: A7TX<-xxxx, A6TX<-xxxx, A5TX<-xxxx, A4TX<-xxxx
    # wreg(0x98d2, 0x0020)    #laneA3-A0: A3TX<-xxxx, A2TX<-xxxx, A1TX<-B2RX, A0TX<-B0RX
    # wreg(0x98d3, 0x0000)    #laneB7-B4: B7TX<-xxxx, B6TX<-xxxx, B5TX<-xxxx, B4TX<-xxxx
    # wreg(0x98d4, 0x1100)    #laneB3-B0: B3TX<-A1RX, B2TX<-A1RX, B1TX<-A0RX, B0TX<-A0RX 
    
    if all(elem in A_lanes for elem in group0_100G):  # If A_lanes contains [0,1]
        wreg([0x98d2,[ 7, 4]], 2) # A1TX<-B2RX
        wreg([0x98d2,[ 3, 0]], 0) # A0TX<-B0RX
        wreg([0x98d4,[15,12]], 1) # B3TX<-A1RX 
        wreg([0x98d4,[11, 8]], 1) # B2TX<-A1RX 
        wreg([0x98d4,[ 7, 4]], 0) # B1TX<-A0RX 
        wreg([0x98d4,[ 3, 0]], 0) # B0TX<-A0RX 
        wreg([0x9858,  [1,0]], 0) # FEC_A0 always uses lane A0 (Line mapping for gearbox, FEC_A0 (for 100G-2, A0/A1<->B0/B1/B2/B3)
        wreg([0x9857,    [0]], 1) # FEC_A0 Enabled  (for 100G-2 lanes A0/A1 to B0/B1/B2/B3)
        if fec_b_byp==False:
            if print_en: print("\n\n...Gearbox 100G-2     : Setting Up Lanes A0/A1 to B0/B1/B2/B3 with FEC_A0/FEC_B0"),
            wreg([0x9857,    [4]], 1) # FEC_B0 Enabled  (for 100G-2 lanes A0/A1 to B0/B1/B2/B3)
        else:
            if print_en: print("\n\n...Gearbox 100G-2     : Setting Up Lanes A0/A1 to B0/B1/B2/B3 with FEC_A0 (FEC_B0 Bypassed)"),
        FEC_A_IDX = 0 # FEC_A0
        FEC_B_IDX = 4 # FEC_B0
        sw_config_fec_100G(FEC_A_IDX, FEC_B_IDX)
        
    if all(elem in A_lanes for elem in group1_100G):  # If A_lanes contains [2,3]
        wreg([0x98d2,[15,12]], 6) # A3TX<-B6RX
        wreg([0x98d2,[11, 8]], 4) # A2TX<-B4RX
        wreg([0x98d3,[15,12]], 3) # B7TX<-A3RX 
        wreg([0x98d3,[11, 8]], 3) # B6TX<-A3RX 
        wreg([0x98d3,[ 7, 4]], 2) # B5TX<-A2RX 
        wreg([0x98d3,[ 3, 0]], 2) # B4TX<-A2RX 
        wreg([0x9858,  [5,4]], 2) # FEC_A2=2, uses lanes A2/A3 (Line mapping for gearbox, FEC_A2 (for 100G-2, A2/A3 to B4/B5/B6/B7)
        wreg([0x9857,    [2]], 1) # FEC_A2 Enabled  (for 100G-2 lanes A2/A3 to B4/B5/B6/B7)
        if fec_b_byp==False:
            if print_en: print("\n\n...Gearbox 100G-2     : Setting Up Lanes A2/A3 to B4/B5/B6/B7 with FEC_A2/FEC_B2"),
            wreg([0x9857,    [6]], 1) # FEC_B2 Enabled  (for 100G-2 lanes A2/A3 to B4/B5/B6/B7)
        else:
            if print_en: print("\n\n...Gearbox 100G-2     : Setting Up Lanes A2/A3 to B4/B5/B6/B7 with FEC_A2 (FEC_B2 Bypassed)"),
        FEC_A_IDX = 2 # FEC_A2
        FEC_B_IDX = 6 # FEC_B2
        sw_config_fec_100G(FEC_A_IDX, FEC_B_IDX)
        
    elif all(elem in A_lanes for elem in group2_100G): # If A_lanes contains [4,5]
        wreg([0x98d1,[ 7, 4]], 6) # A5TX<-B6RX
        wreg([0x98d1,[ 3, 0]], 4) # A4TX<-B4RX
        wreg([0x98d3,[15,12]], 5) # B7TX<-A5RX 
        wreg([0x98d3,[11, 8]], 5) # B6TX<-A5RX 
        wreg([0x98d3,[ 7, 4]], 4) # B5TX<-A4RX 
        wreg([0x98d3,[ 3, 0]], 4) # B4TX<-A4RX 
        wreg([0x9858,  [5,4]], 0) # FEC_A2=0, uses lanes A4/A5 (Line mapping for gearbox, FEC_A2 (for 100G-2, A4/A5 to B4/B5/B6/B7)
        wreg([0x9857,    [2]], 1) # FEC_A2 Enabled  (for 100G-2 lanes A4/A5 to B4/B5/B6/B7)
        if fec_b_byp==False:
            if print_en: print("\n\n...Gearbox 100G-2    : Setting Up Lanes A4/A5 to B4/B5/B6/B7 with FEC_A2/FEC_B2"),
            wreg([0x9857,    [6]], 1) # FEC_B2 Enabled  (for 100G-2 lanes A4/A5 to B4/B5/B6/B7)
        else:
            if print_en: print("\n\n...Gearbox 100G-2    : Setting Up Lanes A4/A5 to B4/B5/B6/B7 with FEC_A2 (FEC_B2 Bypassed)"),
        
        FEC_A_IDX = 2 # FEC_A2
        FEC_B_IDX = 6 # FEC_B2
        sw_config_fec_100G(FEC_A_IDX, FEC_B_IDX)

    
    #fec_status()
    
    #### FEC's Line mapping for gearbox or bitmux modes####
   #wreg(0x9858, 0x0000)    # FEC_A0, FEC_A2 (for 100G-2, A0/A1<->B0/B1/B2/B3 and  A4/A5<->B4/B5/B6/B7)
   #wreg(0x9858, 0x0020)    # FEC_A0, FEC_A2 (for 100G-2, A0/A1<->B0/B1/B2/B3 and  A2/A3<->B4/B5/B6/B7)
   #wreg(0x9858, 0x0008)    # FEC_A0, FEC_A1 (for  50G-1, A0<->B0/B1 and A1<->B2/B3))
   #wreg(0x9858, 0x0028)    # FEC_A0, FEC_A1, FEC_A2, FEC_A3 (for  50G-1, A0<->B0/B1,  A1<->B2/B3,  A2<->B4/B5,  A3<->B6/B7 )


    ####FEC################## FEC_A   FEC_B   (gearbox mode)    
   #wreg(0x9857, 0x0001)    # FEC_A0,   off   (no FEC on NRZ side)
   #wreg(0x9857, 0x0011)    # FEC_A0, FEC_B0  (for 100G-2 lanes A0/A1 to B0/B1/B2/B3)
   #wreg(0x9857, 0x0044)    # FEC_A2, FEC_B2  (for 100G-2 lanes A2/A3 to B4/B5/B6/B7)
  
#############################################################################  
# Set PAM4-NRZ Lanes in Gear Box mode, A-side PAM4, B-side: NRZ
#
#
##############################################################################
   
def sw_config_gearbox_50G(A_lanes=[0], fec_b_byp=False):
    
    print("\n...SW Gearbox 50G-1!")
    print_en=1
       
    # For 50G-2 Gearbox mode, 3 options supported for A-Lane groups 
    group0_50G=[0] # A_lanes group 1 -> [A0] <-> [ 8, 9]
    group1_50G=[1] # A_lanes group 2 -> [A1] <-> {10,11]
    group2_50G=[2] # A_lanes group 3 -> [A2] <-> [12,13]
    group3_50G=[3] # A_lanes group 4 -> [A3] <-> [14,15]
    
    #Determine the corresponding B-Lanes for each group of A-Lanes
    B_lanes=[]
    if all(elem in A_lanes for elem in group0_50G): # If A_lanes contains [0]
        B_lanes+=[ 8, 9]                        
    if all(elem in A_lanes for elem in group1_50G): # If A_lanes contains [1]
        B_lanes+=[10,11]                      
    if all(elem in A_lanes for elem in group2_50G): # If A_lanes contains [2]
        B_lanes+=[12,13]
    if all(elem in A_lanes for elem in group3_50G): # If A_lanes contains [3]
        B_lanes+=[14,15]
    #else:
    #    print("\n*** 50G-1 Gearbox Setup: Invalid Target A-Lanes specified!\n")
    #    return
    
    lanes = sorted(list(set(A_lanes + B_lanes)))
    prbs_mode_select(lane=lanes, prbs_mode='functional')
    
    for ln in A_lanes: ################# PAM4 lanes general setup for gearbox mode
        wreg(0x3000+0x100*ln,0x0000,0) # LT dis
        wreg([0x079,[12,0]],0x1454,ln) # PAM4 TRF for 50G-to-25G
        wreg([0x0f4,[0]],        1,ln) # TXPLL use RX Recovered 
        wreg([0x0f3,[2]],        0,ln) # TX phase rotator flip
        wreg([0x0f3,[15]],       1,ln) # TX phase rotator en
        wreg([0x0FB,[0]],        1,ln) # TX FEC Clock en
        wreg([0x0F1,[3]],        1,ln) # RX FEC Clock en 
        
    for ln in B_lanes: ################# NRZ lanes general setup for gearbox mode
        wreg(0x3000+0x100*ln,0x0000,0) # LT dis
        wreg([0x17a,[12,0]],0x114C,ln) # NRZ TRF for 25G-to-50G
        wreg([0x0f4,[0]],        1,ln) # TXPLL use RX Recovered 
        wreg([0x0f3,[2]],        0,ln) # TX phase rotator flip
        wreg([0x0f3,[15]],       1,ln) # TX phase rotator en
        wreg([0x0FB,[0]],        1,ln) # TX FEC Clock en        
        wreg([0x0F1,[3]],        1,ln) # RX FEC Clock en
          
    if all(elem in A_lanes for elem in group0_50G):  # If A_lanes contains [0]
        wreg([0x98d2,[ 3, 0]], 0) # A0TX<-B0RX
        wreg([0x98d4,[ 7, 4]], 0) # B1TX<-A0RX 
        wreg([0x98d4,[ 3, 0]], 0) # B0TX<-A0RX 
        wreg([0x9858,  [1,0]], 0) # FEC_A0 always uses lane A0 (Line mapping for gearbox, FEC_A0 (for 50G-1, A0<->B0/B1)
        wreg([0x9857,    [0]], 1) # FEC_A0 Enabled  (for 50G-1 lanes A0 to B0/B1)
        wreg([0x9857,    [8]], 1) # FEC_A0 Enabled  (for 50G-1 lanes A0 to B0/B1)
        if fec_b_byp==False:
            if print_en: print("\n\n...Gearbox 50G-1     : Setting Up Lanes A0 to B0/B1 with FEC_A0/FEC_B0"),
            wreg([0x9857,    [4]], 1) # FEC_B0 Enabled  (for 50G-1 lanes A0 to B0/B1)
        else:
            if print_en: print("\n\n...Gearbox 50G-1     : Setting Up Lanes A0 to B0/B1 with FEC_A0 (FEC_B0 Bypassed)"),
        FEC_A_IDX = 0 # FEC_A0
        FEC_B_IDX = 4 # FEC_B0
        sw_config_fec_50G(FEC_A_IDX, FEC_B_IDX)
        
    if all(elem in A_lanes for elem in group1_50G):  # If A_lanes contains [1]
        wreg([0x98d2,[ 7, 4]], 2) # A1TX<-B2RX
        wreg([0x98d4,[15,12]], 1) # B3TX<-A1RX 
        wreg([0x98d4,[11, 8]], 1) # B2TX<-A1RX 
        wreg([0x9858,  [3,2]], 2) # FEC_A1=1, uses lanes A1 (Line mapping for gearbox, FEC_A1 (for 50G-1, A1 to B2/B3)
        wreg([0x9857,    [1]], 1) # FEC_A1 Enabled  (for 50G-1 lanes A1 to B2/B3)
        wreg([0x9857,    [9]], 1) # FEC_A1 Enabled  (for 50G-1 lanes A1 to B2/B3)
        if fec_b_byp==False:
            if print_en: print("\n\n...Gearbox 50G-1     : Setting Up Lanes A1 to B2/B3 with FEC_A1/FEC_B1"),
            wreg([0x9857,    [5]], 1) # FEC_B1 Enabled  (for 50G-1 lanes A1 to B2/B3)
        else:
            if print_en: print("\n\n...Gearbox 50G-1     : Setting Up Lanes A1 to B2/B3 with FEC_A1 (FEC_B1 Bypassed)"),
        FEC_A_IDX = 1 # FEC_A1
        FEC_B_IDX = 5 # FEC_B1
        sw_config_fec_50G(FEC_A_IDX, FEC_B_IDX)

    if all(elem in A_lanes for elem in group2_50G):  # If A_lanes contains [2]
        wreg([0x98d2,[11, 8]], 4) # A2TX<-B4RX
        wreg([0x98d3,[ 7, 4]], 2) # B5TX<-A2RX 
        wreg([0x98d3,[ 3, 0]], 2) # B4TX<-A2RX 
        wreg([0x9858,  [5,4]], 2) # FEC_A2=2, uses lanes A2 (Line mapping for gearbox, FEC_A2 (for 50G-1, A2 to B4/B5)
        wreg([0x9857,    [2]], 1) # FEC_A2 Enabled  (for 50G-1 lanes A2 to B4/B5)
        wreg([0x9857,   [10]], 1) # FEC_A2 Enabled  (for 50G-1 lanes A2 to B4/B5)
        if fec_b_byp==False:
            if print_en: print("\n\n...Gearbox 50G-1     : Setting Up Lanes A2 to B4/B5 with FEC_A2/FEC_B2"),
            wreg([0x9857,    [6]], 1) # FEC_B2 Enabled  (for 50G-1 lanes A2 to B4/B5)
        else:
            if print_en: print("\n\n...Gearbox 50G-1     : Setting Up Lanes A2 to B4/B5 with FEC_A2 (FEC_B2 Bypassed)"),
        FEC_A_IDX = 2 # FEC_A2
        FEC_B_IDX = 6 # FEC_B2
        sw_config_fec_50G(FEC_A_IDX, FEC_B_IDX)
        
    if all(elem in A_lanes for elem in group3_50G):  # If A_lanes contains [3]
        wreg([0x98d2,[15,12]], 6) # A3TX<-B6RX
        wreg([0x98d3,[15,12]], 3) # B7TX<-A3RX 
        wreg([0x98d3,[11, 8]], 3) # B6TX<-A3RX 
        wreg([0x9858,  [7,6]], 2) # FEC_A3=2, uses lanes A3 (Line mapping for gearbox, FEC_A3 (for 50G-1, A2 to B6/B7)
        wreg([0x9857,    [3]], 1) # FEC_A3 Enabled  (for 50G-1 lanes A3 to B6/B7)
        wreg([0x9857,   [11]], 1) # FEC_A3 Enabled  (for 50G-1 lanes A3 to B6/B7)
        if fec_b_byp==False:
            if print_en: print("\n\n...Gearbox 50G-1     : Setting Up Lanes A3 to B6/B7 with FEC_A3/FEC_B3"),
            wreg([0x9857,    [7]], 1) # FEC_B3 Enabled  (for 50G-1 lanes A3 to B4/B5)
        else:
            if print_en: print("\n\n...Gearbox 50G-1     : Setting Up Lanes A3 to B6/B7 with FEC_A3 (FEC_B3 Bypassed)"),
        FEC_A_IDX = 3 # FEC_A3
        FEC_B_IDX = 7 # FEC_B3
        sw_config_fec_50G(FEC_A_IDX, FEC_B_IDX)

  
##############################################################################
def fec_reset(fec_list=[0,1,2,3,4,5,6,7]):

    for fec_idx in fec_list:
        wreg([0x9857,   [fec_idx]], 1) # FEC_x enabled
        time.sleep(0.01)
        wreg([fec_class3[fec_idx]+0x80,[7,6]], 0x3) # Reset FEC x
        time.sleep(0.01)
        wreg([fec_class3[fec_idx]+0x80,[11,8]], 0xf) # FEC_A: TX forced to value 1, RX forced to reset
        wreg([fec_class3[fec_idx]+0x80,[11,8]], 0xf) # FEC_B: TX forced to value 1, RX forced to reset
        wreg([0x9857,   [fec_idx]], 0) # FEC_x disabled
  
    #wreg(0x9858, 0x0000) # clear FEC-to-SerdesLanes Mapping
    
##############################################################################  
# Configure FEC_A/FEC_B, for 100G Gear Box mode, A-side PAM4, B-side: NRZ
#
#                   
#
##############################################################################
def sw_config_fec_50G(FEC_A_IDX=0, FEC_B_IDX=4):

    print_en=1
    if print_en: 
        if   FEC_A_IDX==0: print("\n...Gearbox 50G-1 AtoB: Setting up FEC_A0 first and FEC_B0 next........."),
        elif FEC_A_IDX==1: print("\n...Gearbox 50G-1 AtoB: Setting up FEC_A1 first and FEC_B1 next........."),
        elif FEC_A_IDX==2: print("\n...Gearbox 50G-1 AtoB: Setting up FEC_A2 first and FEC_B2 next........."),
        elif FEC_A_IDX==3: print("\n...Gearbox 50G-1 AtoB: Setting up FEC_A3 first and FEC_B3 next........."),

        
    fec_line_mapping= rreg(0x9858) # save this register to restore after FEC Reset
    fec_enable_bits = rreg(0x9857) # save this register to restore after FEC Reset
    
    wregBits(fec_class3[FEC_A_IDX]+0x80,[7,6], 0x3) # Reset FEC A Registers
    wregBits(fec_class3[FEC_B_IDX]+0x80,[7,6], 0x3) # Reset FEC B Registers
    time.sleep(.1)

    wreg(0x9858, fec_line_mapping) # FEC's Line mapping for gearbox, FEC_A0 and FEC_A2 (for 100G-2, A0/A1<->B0/B1/B2/B3 and  A4/A5<->B4/B5/B6/B7)
    wreg(0x9857, fec_enable_bits)  # FEC Enable: FEC_A2, FEC_B2  (for 100G-2 lanes A4/A5 to B4/B5/B6/B7)

    ###need set it manually
    #wregBits(fec_class3[FEC_A_IDX]+0x50,[12], 0x1) # for debug only. comment out for normal operation 
    #wregBits(fec_class3[FEC_B_IDX]+0x50,[12], 0x1) # for debug only. comment out for normal operation 
    #wreg(0x4850, 0x1003);                         # for debug only. comment out for normal operation 
    #wreg(0x5850, 0x1003);                         # for debug only. comment out for normal operation 

    wregBits(fec_class3[FEC_A_IDX]+0x80,[11,8], 0xF) # FEC_A: TX forced to value 1, RX forced to reset
    wregBits(fec_class3[FEC_B_IDX]+0x80,[11,8], 0xF) # FEC_B: TX forced to value 1, RX forced to reset
    wregBits(fec_class3[FEC_A_IDX]+0x80,   [4], 0x1) # FEC A AM16 enabled
    wregBits(fec_class3[FEC_B_IDX]+0x80,   [4], 0x1) # FEC B AM16 enabled
    #wreg(0x4880, 0x0f00);   
    #wreg(0x5880, 0x0f00);

    #time.sleep(8)   # Wait for CTLE_DONE      # A-side. Wait to allow PAM4 RX to adapt before removing reset of FEC_A RX
    if print_en: print("\n...Gearbox 50G-1 AtoB: Waiting for A Lanes RX Adaptation to Complete..."),
    if   FEC_A_IDX==0: fw_adapt_wait (max_wait=8.0, lane=[0], print_en=0)
    elif FEC_A_IDX==1: fw_adapt_wait (max_wait=8.0, lane=[1], print_en=0)
    elif FEC_A_IDX==2: fw_adapt_wait (max_wait=8.0, lane=[2], print_en=0)
    elif FEC_A_IDX==3: fw_adapt_wait (max_wait=8.0, lane=[3], print_en=0)
    if print_en: print("Done!"),
    
    if print_en: print("\n...Gearbox 50G-1 AtoB: Waiting for FEC_A Lock to Complete.............."),
    wreg([fec_class3[FEC_A_IDX]+0x80,[ 8]], 0) # FEC_A RX Reset removed. Data flowing to FEC_B TX
    time.sleep(.1) # wait for FECA lock 0x40c9=0x4303    # Wait before turning on FEC_B TX. Let FEC_A Decoder DATA_OUT to reach FEC_B
    wreg([fec_class3[FEC_B_IDX]+0x80,[10]], 0) # FEC_B TX Reset removed, TX-B output is enabled
    time.sleep(.1)
    if print_en: print("Done!"),
        
    #time.sleep(1)   # Wait for CTLE_DONE      # B-side. Wait to allow NRZ RX to adapt before removing reset of FEC_A RX
    if print_en: print("\n...Gearbox 50G-1 BtoA: Waiting for B Lanes RX Adaptation to Complete..."),
    if   FEC_B_IDX==4: fw_adapt_wait (max_wait=1.0, lane=[ 8,  9], print_en=0)
    elif FEC_B_IDX==5: fw_adapt_wait (max_wait=1.0, lane=[ 10,11], print_en=0)
    elif FEC_B_IDX==6: fw_adapt_wait (max_wait=1.0, lane=[ 12,13], print_en=0)
    elif FEC_B_IDX==7: fw_adapt_wait (max_wait=1.0, lane=[ 14,15], print_en=0)
    if print_en: print("Done!"),
    
    if print_en: print("\n...Gearbox 50G-1 BtoA: Waiting for FEC_B Lock to Complete.............."),
    time.sleep(.1) # Wait for CTLE_DONE        # wait before turning on FEC_B RX
    wreg([fec_class3[FEC_B_IDX]+0x80,[ 8]], 0) # FEC_B RX Reset removed.  Data flowing to FEC_A TX
    time.sleep(.1)  # wait for FECB lock 0x50c9=0x4F03   # Wait before turning on FEC_A TX. Let FEC_B RX DATA_OUT to reach FEC_A
    wreg([fec_class3[FEC_A_IDX]+0x80,[10]], 0) # FEC_A TX Reset removed,, TX-A output is enabled
    if print_en: print("Done!"),


    
    #wreg([fec_class3[FEC_B_IDX]+0x80,[ 4]], 1) # Set this bit to '1' if working with Enigma
    
    #expecting FEC lock condition here: 0x40c9=0x4f03
##############################################################################  
# Configure FEC_A/FEC_B, for 100G Gear Box mode, A-side PAM4, B-side: NRZ
#
#                   
#
##############################################################################
def sw_config_fec_100G_AtoB(FEC_A_IDX=0, FEC_B_IDX=4):

    print_en=1
    if print_en: 
        if FEC_A_IDX==0: print("\n...Gearbox 100G-2 BtoA: Setting up FEC_A0 first and FEC_B0 next........."),
        else:            print("\n...Gearbox 100G-2 BtoA: Setting up FEC_A2 first and FEC_B2 next........."),
        
    fec_line_mapping= rreg(0x9858) # save this register to restore after FEC Reset
    fec_enable_bits = rreg(0x9857) # save this register to restore after FEC Reset
    
    wregBits(fec_class3[FEC_A_IDX]+0x80,[7,6], 0x3) # Reset FEC A Registers
    wregBits(fec_class3[FEC_B_IDX]+0x80,[7,6], 0x3) # Reset FEC B Registers
    time.sleep(.1)

    wreg(0x9858, fec_line_mapping) # FEC's Line mapping for gearbox, FEC_A0 and FEC_A2 (for 100G-2, A0/A1<->B0/B1/B2/B3 and  A4/A5<->B4/B5/B6/B7)
    wreg(0x9857, fec_enable_bits)  # FEC Enable: FEC_A2, FEC_B2  (for 100G-2 lanes A4/A5 to B4/B5/B6/B7)

    ###need set it manually
    #wregBits(fec_class3[FEC_A_IDX]+0x50,[12], 0x1) # for debug only. comment out for normal operation 
    #wregBits(fec_class3[FEC_B_IDX]+0x50,[12], 0x1) # for debug only. comment out for normal operation 
    #wreg(0x4850, 0x1003);                         # for debug only. comment out for normal operation 
    #wreg(0x5850, 0x1003);                         # for debug only. comment out for normal operation 

    wregBits(fec_class3[FEC_A_IDX]+0x80,[11,8], 0xF) # FEC_A: TX forced to value 1, RX forced to reset
    wregBits(fec_class3[FEC_B_IDX]+0x80,[11,8], 0xF) # FEC_B: TX forced to value 1, RX forced to reset
    wregBits(fec_class3[FEC_A_IDX]+0x80,   [4], 0x1) # FEC A AM16 enabled
    wregBits(fec_class3[FEC_B_IDX]+0x80,   [4], 0x1) # FEC B AM16 enabled
    #wreg(0x4880, 0x0f00);   
    #wreg(0x5880, 0x0f00);

    #time.sleep(8)   # Wait for CTLE_DONE      # A-side. Wait to allow PAM4 RX to adapt before removing reset of FEC_A RX
    if print_en: print("\n...Gearbox 100G-2 AtoB: Waiting for A Lanes RX Adaptation to Complete..."),
    if FEC_A_IDX==0: fw_adapt_wait (max_wait=8.0, lane=[0,1], print_en=0)
    else:            fw_adapt_wait (max_wait=8.0, lane=[2,3], print_en=0)
    if print_en: print("Done!"),
    
    if print_en: print("\n...Gearbox 100G-2 AtoB: Waiting for FEC_A Lock to Complete.............."),
    wreg([fec_class3[FEC_A_IDX]+0x80,[ 8]], 0) # FEC_A RX Reset removed. Data flowing to FEC_B TX
    time.sleep(2) # wait for FECA lock 0x40c9=0x4F03    # Wait before turning on FEC_B TX. Let FEC_A Decoder DATA_OUT to reach FEC_B
    wreg([fec_class3[FEC_B_IDX]+0x80,[10]], 0) # FEC_B TX Reset removed, TX-B output is enabled
    time.sleep(.1)
    if print_en: print("Done!"),
        
    #time.sleep(1)   # Wait for CTLE_DONE      # B-side. Wait to allow NRZ RX to adapt before removing reset of FEC_A RX
    if print_en: print("\n...Gearbox 100G-2 BtoA: Waiting for B Lanes RX Adaptation to Complete..."),
    if FEC_B_IDX==4: fw_adapt_wait (max_wait=2.0, lane=[ 8, 9,10,11], print_en=0)
    else:            fw_adapt_wait (max_wait=2.0, lane=[12,13,14,15], print_en=0)
    if print_en: print("Done!"),
    
    if print_en: print("\n...Gearbox 100G-2 BtoA: Waiting for FEC_B Lock to Complete.............."),
    time.sleep(.1) # Wait for CTLE_DONE        # wait before turning on FEC_B RX
    wreg([fec_class3[FEC_B_IDX]+0x80,[ 8]], 0) # FEC_B RX Reset removed.  Data flowing to FEC_A TX
    time.sleep(1)  # wait for FECB lock 0x50c9=0x4F03   # Wait before turning on FEC_A TX. Let FEC_B RX DATA_OUT to reach FEC_A
    wreg([fec_class3[FEC_A_IDX]+0x80,[10]], 0) # FEC_A TX Reset removed,, TX-A output is enabled
    if print_en: print("Done!"),


    
    #wreg([fec_class3[FEC_B_IDX]+0x80,[ 4]], 1) # Set this bit to '1' if working with Enigma
    
    #expecting FEC lock condition here: 0x40c9=0x4f03
##############################################################################  
# Configure FEC_A/FEC_B, for 100G Gear Box mode, A-side PAM4, B-side: NRZ
#
#                   
#
##############################################################################
def sw_config_fec_100G_BtoA(FEC_A_IDX=0, FEC_B_IDX=4):

    print_en=1
    if print_en: 
        if FEC_A_IDX==0: print("\n...Gearbox 100G-2 BtoA: Setting up FEC_B0 first and FEC_A0 next........."),
        else:            print("\n...Gearbox 100G-2 BtoA: Setting up FEC_B2 first and FEC_A2 next........."),
        
    fec_line_mapping= rreg(0x9858) # save this register to restore after FEC Reset
    fec_enable_bits = rreg(0x9857) # save this register to restore after FEC Reset
    
    wregBits(fec_class3[FEC_A_IDX]+0x80,[7,6], 0x3) # Reset FEC A Registers
    wregBits(fec_class3[FEC_B_IDX]+0x80,[7,6], 0x3) # Reset FEC B Registers
    time.sleep(.1)

    wreg(0x9858, fec_line_mapping) # FEC's Line mapping for gearbox, FEC_A0 and FEC_A2 (for 100G-2, A0/A1<->B0/B1/B2/B3 and  A4/A5<->B4/B5/B6/B7)
    wreg(0x9857, fec_enable_bits)  # FEC Enable: FEC_A2, FEC_B2  (for 100G-2 lanes A4/A5 to B4/B5/B6/B7)

    ###need set it manually
    #wregBits(fec_class3[FEC_A_IDX]+0x50,[12], 0x1) # for debug only. comment out for normal operation 
    #wregBits(fec_class3[FEC_B_IDX]+0x50,[12], 0x1) # for debug only. comment out for normal operation 
    #wreg(0x4850, 0x1003);                         # for debug only. comment out for normal operation 
    #wreg(0x5850, 0x1003);                         # for debug only. comment out for normal operation 

    wregBits(fec_class3[FEC_A_IDX]+0x80,[11,8], 0xF) # FEC_A: TX forced to value 1, RX forced to reset
    wregBits(fec_class3[FEC_B_IDX]+0x80,[11,8], 0xF) # FEC_B: TX forced to value 1, RX forced to reset
    wregBits(fec_class3[FEC_A_IDX]+0x80,   [4], 0x1) # FEC A AM16 enabled
    wregBits(fec_class3[FEC_B_IDX]+0x80,   [4], 0x1) # FEC B AM16 enabled
    #wreg(0x4880, 0x0f00);   
    #wreg(0x5880, 0x0f00);
    
    #time.sleep(1)   # Wait for CTLE_DONE      # B-side. Wait to allow NRZ RX to adapt before removing reset of FEC_A RX
    if print_en: print("\n...Gearbox 100G-2 BtoA: Waiting for B Lanes RX Adaptation to Complete..."),
    if FEC_B_IDX==4: fw_adapt_wait (max_wait=4.0, lane=[ 8, 9,10,11], print_en=0)
    else:            fw_adapt_wait (max_wait=4.0, lane=[12,13,14,15], print_en=0)
    if print_en: print("Done!"),
    
    if print_en: print("\n...Gearbox 100G-2 BtoA: Waiting for FECB Lock to Complete..............."),
    time.sleep(.1) # Wait for CTLE_DONE        # wait before turning on FEC_B RX
    wreg([fec_class3[FEC_B_IDX]+0x80,[ 8]], 0) # FEC_B RX Reset removed.  Data flowing to FEC_A TX
    time.sleep(1)  # wait for FECB lock 0x50c9=0x4F03   # Wait before turning on FEC_A TX. Let FEC_B RX DATA_OUT to reach FEC_A
    wreg([fec_class3[FEC_A_IDX]+0x80,[10]], 0) # FEC_A TX Reset removed,, TX-A output is enabled
    if print_en: print("Done!"),

    #time.sleep(8)   # Wait for CTLE_DONE      # A-side. Wait to allow PAM4 RX to adapt before removing reset of FEC_A RX
    if print_en: print("\n...Gearbox 100G-2 AtoB: Waiting for A Lanes RX Adaptation to Complete..."),
    if FEC_A_IDX==0: fw_adapt_wait (max_wait=10.0, lane=[0,1], print_en=0)
    else:            fw_adapt_wait (max_wait=10.0, lane=[2,3], print_en=0)
    if print_en: print("Done!"),
    
    if print_en: print("\n...Gearbox 100G-2 AtoB: Waiting for FECA Lock to Complete..............."),
    wreg([fec_class3[FEC_A_IDX]+0x80,[ 8]], 0) # FEC_A RX Reset removed. Data flowing to FEC_B TX
    time.sleep(2) # wait for FECA lock 0x40c9=0x4F03    # Wait before turning on FEC_B TX. Let FEC_A Decoder DATA_OUT to reach FEC_B
    wreg([fec_class3[FEC_B_IDX]+0x80,[10]], 0) # FEC_B TX Reset removed, TX-B output is enabled
    time.sleep(.1)
    if print_en: print("Done!"),
    
    wreg([fec_class3[FEC_B_IDX]+0x80,[ 4]], 1) # Set this bit to '1' if working with Enigma
    
    #expecting FEC lock condition here: 0x40c9=0x4f03
                  

#sw_config_fec_100G = sw_config_fec_100G_BtoA
sw_config_fec_100G = sw_config_fec_100G_AtoB
    
####################################################################################################  
# Set PAM4-NRZ Lanes in Gear Box mode, A-side PAM4, B-side: NRZ
#
# 2x25G NRZ  B1-B0 
# 1x50G PAM4 A0
#
####################################################################################################
def sw_config_gearbox_50G_orig(A_lanes=[0], B_lanes=[8,9],print_en=1):

    if print_en: print("\nGearbox 50G: Initalize Gearbox Mode..."),  
    
    LT_REG_BASE        = [0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,   0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F]
    TX_B_SIDE_REG_BASE = [0x80,0x82,0x84,0x86,0x88,0x8A,0x8C,0x8E,   0x80,0x82,0x84,0x86,0x88,0x8A,0x8C,0x8E]
    RX_B_SIDE_REG_BASE = [0x81,0x83,0x85,0x87,0x89,0x8B,0x8D,0x8F,   0x81,0x83,0x85,0x87,0x89,0x8B,0x8D,0x8F]
    RX_A_SIDE_REG_BASE = [0x70,0x72,0x78,0x7A,  0x70,0x72,0x78,0x7A, 0x70,0x72,0x78,0x7A,  0x70,0x72,0x78,0x7A]

    lanes = sorted(list(set(A_lanes + B_lanes)))
    prbs_mode_select(lane=lanes, prbs_mode='functional')

    for ln in lanes:
        ###LT disable###
        wreg((LT_REG_BASE[ln]*0x100)+0x00, 0x0000)

        ###TRF Setting###
        wreg((TX_B_SIDE_REG_BASE[ln]*0x100)+0xf4, 0x0001) # NRZ TX intp enable[0]
        wreg((TX_B_SIDE_REG_BASE[ln]*0x100)+0xf3, 0x8004) # NRZ TX reg_phase_rotator_flip_tx[2]
        wreg((RX_B_SIDE_REG_BASE[ln]*0x100)+0x7a, 0x116C) # NRZ TRF setting

        
        wreg((RX_A_SIDE_REG_BASE[ln]*0x100)+0xf4, 0x0001) # PAM4 TX intp enable
        wreg((RX_A_SIDE_REG_BASE[ln]*0x100)+0xf3, 0x8000) # PAM4 phase rotator
        wreg((RX_A_SIDE_REG_BASE[ln]*0x100)+0x79, 0x2474) # PAM4 TRF setting

        wreg((TX_B_SIDE_REG_BASE[ln]*0x100)+0xf1, 0x000f) # NRZ en_40t_clk_rx B
        wreg((TX_B_SIDE_REG_BASE[ln]*0x100)+0xfb, 0x7fb7) # NRZ en_40t_clk_tx B

        wreg((RX_A_SIDE_REG_BASE[ln]*0x100)+0xf1, 0x000f) # PAM4 en_40t_clk_rx A 
        wreg((RX_A_SIDE_REG_BASE[ln]*0x100)+0xfb, 0x7fb7) # PAM4 en_40t_clk_tx A
        
        #### OVERWRITE some parameters here with newer recommended values
        # wreg((TX_B_SIDE_REG_BASE[ln]*0x100)+0xf3, 0x8004) # NRZ TX reg_phase_rotator_flip_tx[2]
        # wreg((RX_B_SIDE_REG_BASE[ln]*0x100)+0x7a, 0x116c) # 0x116C) # NRZ TRF setting
        
        # wreg((RX_A_SIDE_REG_BASE[ln]*0x100)+0xf3, 0x8000) # PAM4 phase rotator
        # wreg((RX_A_SIDE_REG_BASE[ln]*0x100)+0x79, 0x2494) # 3894) # PAM4 TRF setting
        

    if print_en: print("Done!"), 

    if (0): # original setup with Spirent box
        ###top_tx_pi_sel###
        wreg(0x98d1, 0x0000)  # bit[15:12]: B0_Rx -> A7_Tx
        wreg(0x98d2, 0x0000)  # B0-> A0
        wreg(0x98d3, 0x0000)  #laneB7-B4
        wreg(0x98d4, 0x1100)  # A0->B1  and A0 ->B0

        ####FEC####
        wreg(0x9858, 0x0008) #0101)   #fec_en, 50G-1 lane
        #wreg(0x9857, 0x0202) #0101)   #fec_en on PAM4 side only, 50G-1 lane
        wreg(0x9857, 0x0222) #0101)   #fec_en on both sides, 50G-1 lane
                
        #wreg(0x4980, 0x00c0); # Reset FEC A Registers 
        #wreg(0x5980, 0x00c0); # Reset FEC B Registers 
        time.sleep(.1)
        ###need set it manually
        wreg(0x4850, 0x1003); 
        wreg(0x4950, 0x1003); 
        
        wreg(0x4980, 0x0f00); # Reset FEC A TX and RX
        wreg(0x5980, 0x0f00); # Reset FEC B TX and RX
        
        time.sleep(1)
        wreg(0x4980, 0x0e00); 
        time.sleep(2)
        wreg(0x5980, 0x0b00);
        time.sleep(2)
        wreg(0x5980, 0x0a00); 
        time.sleep(2)
        wreg(0x4980, 0x0a00);
        time.sleep(.1)
        #wreg(0x4880, 0x0000)
        
        # expecting FEC lock condition here: 0x40c9=0x4303 or 0xc303
    elif A_lanes==[0] and B_lanes==[8,9]: # 50G mode with BH
        if print_en: print("\nGearbox 50G: Initalizing FECs on Lanes A0 and B0/B1..."), 

        ###top_tx_pi_sel###
        wreg(0x98d1, 0x0000)  # bit[15:12]: B0_Rx -> A7_Tx
        wreg(0x98d2, 0x0000)  # B0-> A0
        wreg(0x98d3, 0x0000)  #laneB7-B4
        wreg(0x98d4, 0x1100)  # A0->B1  and A0 ->B0
        
        wreg(0x4880, 0x00c0); # Reset FEC A Registers 
        wreg(0x5880, 0x00c0); # Reset FEC B Registers 
        time.sleep(.1)
        
        ####FEC####
        wreg(0x9858, 0x0008) #0101)   #fec_en, 50G-1 lane
        #wreg(0x9857, 0x0202) #0101)   #fec_en on PAM4 side only, 50G-1 lane
        wreg(0x9857, 0x0111) #0101)   #fec_en on both sides, 50G-1 lane, 1.    9857 should be 0111
                
        ###need set it manually
        wreg(0x4850, 0x1003); 
        wreg(0x5850, 0x1003); 

        wreg(0x4880, 0x0f00); # Reset FEC A TX and RX
        wreg(0x5880, 0x0f00); # Reset FEC B TX and RX
        time.sleep(.5)
        wreg(0x4880, 0x0e00); # 3.    Wr 4880=0e00 to release A side FEC Rx, 40c9 should go to 4303, indicating A side FEC Rx has locked to 50G traffic from BH
        time.sleep(2)
        wreg(0x5880, 0x0b00); # 5.    Wr 5880=0b00 to release reset for b side FEC tx, b side B0 B1 rx eye should be good
        time.sleep(2)
        
        # 6. If everything is good(clocks are good due to TRF settings), 
        # register 589c should be toggling in a narrow range, 
        # 589d should be a non-zero value and 589e should be a value < 0x0500.
        
        wreg(0x5880, 0x0a00); # 7.    If 6 is OK, wr 5880=0a00 to release reset for b side FEC rx, reg 50c9 should go to 4303 to indicate B side FEC Rx locks
        time.sleep(2)
        wreg(0x4880, 0x0a00); # 8.    If 7 is OK, wr 4880=0a00 to release A side FEC tx, check registers 489c/d/e, they should have similar behavior as 589c/d/e
        time.sleep(.1)

        #wreg(0x4880, 0x0000)
        #9.    If above is OK, we should be sending 50G traffic back to BH
        # expecting FEC lock condition here: 0x40c9=0x4303 or 0xc303
    
    elif A_lanes==[1] and B_lanes==[10,11]: # 50G mode with BH
        if print_en: print("\nGearbox 50G: Initalizing FECs on Lanes A1 and B2//B3..."), 

        ###top_tx_pi_sel###
        wreg(0x98d1, 0x0000)  # bit[15:12]: B0_Rx -> A7_Tx
        wreg(0x98d2, 0x0020)  # B2-> A1
        wreg(0x98d3, 0x0000)  #laneB7-B4
        wreg(0x98d4, 0x1100)  # A1->B2 and A1 ->B3
        
        wreg(0x4980, 0x00c0); # Reset FEC A Registers 
        wreg(0x5980, 0x00c0); # Reset FEC B Registers 
        time.sleep(.1)
        
        ####FEC####
        wreg(0x9858, 0x0008) #0101)   #fec_en, 50G-1 lane
        #wreg(0x9857, 0x0202) #0101)   #fec_en on PAM4 side only, 50G-1 lane
        wreg(0x9857, 0x0222) #0101)   #fec_en on both sides, 50G-1 lane, 1.    9857 should be 0111
                
        ###need set it manually
        wreg(0x4950, 0x1003); 
        wreg(0x5950, 0x1003); 

        wreg(0x4980, 0x0f00); # Reset FEC A TX and RX
        wreg(0x5980, 0x0f00); # Reset FEC B TX and RX
        time.sleep(.5)
        wreg(0x4980, 0x0e00); # 3.    Wr 4880=0e00 to release A side FEC Rx, 40c9 should go to 4303, indicating A side FEC Rx has locked to 50G traffic from BH
        time.sleep(2)
        wreg(0x5980, 0x0b00); # 5.    Wr 5880=0b00 to release reset for b side FEC tx, b side B0 B1 rx eye should be good
        time.sleep(2)
        
        # 6. If everything is good(clocks are good due to TRF settings), 
        # register 589c should be toggling in a narrow range, 
        # 589d should be a non-zero value and 589e should be a value < 0x0500.
        
        wreg(0x5980, 0x0a00); # 7.    If 6 is OK, wr 5880=0a00 to release reset for b side FEC rx, reg 50c9 should go to 4303 to indicate B side FEC Rx locks
        time.sleep(2)
        wreg(0x4980, 0x0a00); # 8.    If 7 is OK, wr 4880=0a00 to release A side FEC tx, check registers 489c/d/e, they should have similar behavior as 589c/d/e
        time.sleep(.1)

        #wreg(0x4880, 0x0000)
        #9.    If above is OK, we should be sending 50G traffic back to BH
        # expecting FEC lock condition here: 0x40c9=0x4303 or 0xc303
    if print_en: print("Done!"),      
####################################################################################################  
# Set up A-side PAM4 or B-side NRZ LF Traffic pattern generator
#
# 8x25G NRZ  B0-B7
# 4x50G PAM4 A0-A3
#
####################################################################################################
def sw_config_fec_traffic_gen_orig(slice=0, mode='100g', side='A-side',print_en=1):

    if chip_rev(print_en=0) == 1.0:
        print("\n*** FEC Traffic Generator is NOT supported in Chip Rev 1.0 ***")
        return

    #lanes = get_lane_list(lane)
    #lanes = sorted(list(set(A_lanes + B_lanes)))
    slice_power_up_init(slice)
    fw_pause('all dis')
    
    ###### FEC-B Generator, must set up Gearbox A-to-B first before this...
    if 'B' in side.upper():
        if print_en: print("\n B-side NRZ Traffic Generator Setup in"),
        for ln in range(16):
            wreg(0xa0,0x320, ln)            # enable TX functional mode        
        for ln in range(8):
            wreg([0x020,[15,0]],0x0140, ln) # OW Kp Disabled
            #wreg([0x10c,[15,0]],0x8030, ln) # OW Phy Rdy
        
        if ('50' in mode): 
            if print_en: print("50G-1 mode..."), 
            wreg(0x9857, 0x0555)    # Enable FEC B side and enable A side
            wreg(0x9858, 0x00aa)    # FEC-A0/1/2/3 are using lanes A0/1/2/3
            wreg(0x58f0, 0x1094)    # 50G-1 FEC-B Generator setup
            wreg(0x5Af0, 0x1094)    # 50G-1 FEC-B Generator setup
            wreg(0x58fa, 0x0000)    # 50G-1 FEC-B Generator setup
            wreg(0x5Afa, 0x0000)    # 50G-1 FEC-B Generator setup
        else:
            if print_en: print("100G-2 mode..."), 
            wreg(0x9857, 0x0055)    # Enable FEC B side and enable A side
            wreg(0x9858, 0x0020)    # [5:4]='10' FEC-A2 is using lane A2 (i.e. for A0-A3)
            wreg(0x58f0, 0x1095)    # 100G-2 FEC-B Generator setup
            wreg(0x5Af0, 0x1095)    # 100G-2 FEC-B Generator setup
                    
        wreg(0x4880, 0x0f00)    # FEC A Reset 
        wreg(0x4A80, 0x0f00)    # FEC A Reset 
        wreg(0x5880, 0x0f00)    # FEC B Reset 
        wreg(0x5A80, 0x0f00)    # FEC B Reset 
        wreg(0x4880, 0x0e00)    # FEC A RX Remove Reset 
        wreg(0x4A80, 0x0e00)    # FEC A RX Remove Reset 
        wreg(0x5880, 0x0b00)    # FEC B TX Remove Reset 
        wreg(0x5A80, 0x0b00)    # FEC B TX Remove Reset 
        
    ###### FEC-A Generator, , must set up Gearbox B-to-A first before this...
    else:
        init_lane('nrz', lane=range(8,16))
        init_lane('pam4',lane=range(4))
        if print_en: print("\n A-side PAM4 Traffic Generator Setup in"),
        for ln in range(16):
            wreg(0xa0,0x320, ln)            # enable TX functional mode
            wreg([0x13b,[15,0]],0x4000, ln) # OW Kp Disabled
            #wreg([0x10c,[15,0],0x8030, ln) # OW Phy Rdy

        
        if ('50' in mode): 
            if print_en: print("50G-1 mode..."), 
            wreg(0x9857, 0x0555)    # Enable FEC A side and Disable B side
            wreg(0x9858, 0x00aa)    # FEC-A0/1/2/3 are using lanes A0/1/2/3
            wreg(0x48f0, 0x1094)    # 50G-1 FEC-A Generator setup
            wreg(0x4Af0, 0x1094)    # 50G-1 FEC-A Generator setup
            wreg(0x48fa, 0x0000)    # 50G-1 FEC-A Generator setup
            wreg(0x4Afa, 0x0000)    # 50G-1 FEC-A Generator setup
        else:
            if print_en: print("100G-2 mode..."), 
            wreg(0x9857, 0x0055)    # Enable FEC A side and Disable B side
            wreg(0x9858, 0x0020)    # [5:4]='10' FEC-A2 is using lane A2 (i.e. for A0-A3)
            wreg(0x48f0, 0x1095)    # 100G-2 FEC-A Generator setup
            wreg(0x4Af0, 0x1095)    # 100G-2 FEC-A Generator setup
                    
        # wreg(0x4880, 0x0f00)    # FEC A Reset 
        # wreg(0x4A80, 0x0f00)    # FEC A Reset 
        # wreg(0x5880, 0x0f00)    # FEC B Reset 
        # wreg(0x5A80, 0x0f00)    # FEC B Reset 
        # wreg(0x5880, 0x0e00)    # FEC A RX Remove Reset 
        # wreg(0x5A80, 0x0e00)    # FEC A RX Remove Reset 
        # wreg(0x4880, 0x0b00)    # FEC A TX Remove Reset 
        # wreg(0x4A80, 0x0b00)    # FEC A TX Remove Reset 
        sw_config_gearbox_100G(A_lanes=[0,1,2,3], fec_b_byp=False)
        pol(1,1,lane=0)
        pol(1,1,lane=1)
        pol(1,1,lane=2)
        pol(1,1,lane=3)

    if print_en: print("Done!"),
    fec_status()
    
####################################################################################################  
# Set up A-side PAM4 or B-side NRZ LF Traffic pattern generator
#
# 8x25G NRZ  B0-B7
# 4x50G PAM4 A0-A3
#
####################################################################################################
def sw_config_fec_traffic_gen(slice=0, mode='100g', side='A-side',print_en=1):

    if chip_rev(print_en=0) == 1.0:
        print("\n*** FEC Traffic Generator is NOT supported in Chip Rev 1.0 ***")
        return

    #lanes = get_lane_list(lane)
    #lanes = sorted(list(set(A_lanes + B_lanes)))
    slice_power_up_init(slice)
    
    ### Enable Serdes FW to let it cal lane PLLs
    fw_pause('serdes en')
    init_lane('pam4',lane=range(8))
    init_lane('nrz', lane=range(8,16))
    fw_config_lane('pam4',lane=range(8))
    fw_config_lane('nrz',lane=range(8,16))
    time.sleep(0.1)
    ### Disable Serdes FW from now on  
    fw_pause('all dis')
    
    ###### FEC-B Generator, must set up Gearbox A-to-B first before this...
    if 'B' in side.upper():
        if print_en: print("\n B-side NRZ Traffic Generator Setup in"),
        for ln in range(16):
            wreg(0xa0,0x320, ln)            # enable TX functional mode        
        for ln in range(8):
            wreg([0x020,[15,0]],0x0140, ln) # OW Kp Disabled
            #wreg([0x10c,[15,0]],0x8030, ln) # OW Phy Rdy
        
        if ('50' in mode): 
            if print_en: print("50G-1 mode..."), 
            wreg(0x9857, 0x0555)    # Enable FEC B side and enable A side
            wreg(0x9858, 0x00aa)    # FEC-A0/1/2/3 are using lanes A0/1/2/3
            wreg(0x58f0, 0x1094)    # 50G-1 FEC-B Generator setup
            wreg(0x5Af0, 0x1094)    # 50G-1 FEC-B Generator setup
            wreg(0x58fa, 0x0000)    # 50G-1 FEC-B Generator setup
            wreg(0x5Afa, 0x0000)    # 50G-1 FEC-B Generator setup
        else:
            if print_en: print("100G-2 mode..."), 
            wreg(0x9857, 0x0055)    # Enable FEC B side and enable A side
            wreg(0x9858, 0x0020)    # [5:4]='10' FEC-A2 is using lane A2 (i.e. for A0-A3)
            wreg(0x58f0, 0x1095)    # 100G-2 FEC-B Generator setup
            wreg(0x5Af0, 0x1095)    # 100G-2 FEC-B Generator setup
                    
        wreg(0x4880, 0x0f00)    # FEC A Reset 
        wreg(0x4A80, 0x0f00)    # FEC A Reset 
        wreg(0x5880, 0x0f00)    # FEC B Reset 
        wreg(0x5A80, 0x0f00)    # FEC B Reset 
        wreg(0x4880, 0x0e00)    # FEC A RX Remove Reset 
        wreg(0x4A80, 0x0e00)    # FEC A RX Remove Reset 
        wreg(0x5880, 0x0b00)    # FEC B TX Remove Reset 
        wreg(0x5A80, 0x0b00)    # FEC B TX Remove Reset 
        
    ###### FEC-A Generator, , must set up Gearbox B-to-A first before this...
    else:
        A_lanes=[0,1,2,3]
        B_lanes=range(8,16)
        pol(1,1,lane=0)
        pol(1,1,lane=1)
        pol(1,1,lane=2)
        pol(1,1,lane=3)
        pol(1,1,lane=4)
        pol(1,1,lane=5)
        if print_en: print("\n A-side PAM4 Traffic Generator Setup in"),
        for ln in A_lanes:
            wreg(0xf4,0x0000, ln)            # disable TRF
            wreg(0xa0,0x2320, ln)            # enable TX functional mode
        for ln in B_lanes:
            wreg(0xf4,0x0000, ln)            # disable TRF
            wreg(0xa0,0x0320, ln)            # enable TX functional mode
            wreg([0x13b,[15,0]],0x4000, ln) # OW Kp Disabled on B-side
            #wreg([0x10c,[15,0],0x8030, ln) # OW Phy Rdy on B-side
       
        if ('50' in mode): 
            if print_en: print("50G-1 mode..."), 
            wreg(0x9857, 0x0FFF)    # Enable FEC A side and Disable B side
            wreg(0x9858, 0x00aa)    # FEC-A0/1/2/3 are using lanes A0/1/2/3
            wreg(0x48f0, 0x1094)    # 50G-1 FEC-A Generator setup
            wreg(0x4Af0, 0x1094)    # 50G-1 FEC-A Generator setup
            wreg(0x48fa, 0x0000)    # 50G-1 FEC-A Generator setup
            wreg(0x4Afa, 0x0000)    # 50G-1 FEC-A Generator setup
            sw_config_gearbox_50G(A_lanes=A_lanes)            
        else:
            if print_en: print("100G-2 mode..."), 
            wreg(0x9857, 0x0055)    # Enable FEC A side and Disable B side
            wreg(0x9858, 0x0020)    # [5:4]='10' FEC-A2 is using lane A2 (i.e. for A0-A3)
            wreg(0x48f0, 0x1095)    # 100G-2 FEC-A Generator setup
            wreg(0x4Af0, 0x1095)    # 100G-2 FEC-A Generator setup
            sw_config_gearbox_100G(A_lanes=A_lanes)            
                    
        # wreg(0x4880, 0x0f10)    # FEC A Reset 
        # wreg(0x4A80, 0x0f10)    # FEC A Reset 
        # wreg(0x5880, 0x0f10)    # FEC B Reset 
        # wreg(0x5A80, 0x0f10)    # FEC B Reset 
        # wreg(0x5880, 0x0a10)    # FEC A RX Remove Reset 
        # wreg(0x5A80, 0x0a10)    # FEC A RX Remove Reset 
        # wreg(0x4880, 0x0a10)    # FEC A TX Remove Reset 
        # wreg(0x4A80, 0x0a10)    # FEC A TX Remove Reset 
        #sw_config_gearbox_100G(A_lanes=[0,1,2,3], fec_b_byp=False)
        for ln in range(16):
            wreg(0xf4,0x0000, ln)            # disable TRF
        for ln in A_lanes:
            wreg(0xa0,0x2320, ln)            # enable TX functional mode
        time.sleep(.2)
        for ln in A_lanes:
            wreg(0xa0,0x0320, ln)            # enable TX functional mode
        pol(1,1,lane=0)
        pol(1,1,lane=1)
        pol(1,1,lane=2)
        pol(1,1,lane=3)
        pol(1,1,lane=4)
        pol(1,1,lane=5)
        
    fec_status()
   
####################################################################################################  
# Set PAM4-NRZ Lanes in Gear Box mode, A-side PAM4, B-side: NRZ
#
# Examples:
# A_lanes=[0], B_lanes=[ 8, 9]: 1x50G PAM4 A0 <-> 2x25G NRZ  B1-B0
# A_lanes=[1], B_lanes=[10,11]: 1x50G PAM4 A1 <-> 2x25G NRZ  B2-B3
#
####################################################################################################
def fec_monitor_setup(enable='en', A_lanes=[0], B_lanes=[8,9], print_en=1):

    #if (rst==1): +
    #   fec_monitor_clear(lane=lane)

    lanes = sorted(list(set(A_lanes + B_lanes)))
    #get_lane_mode(lanes) # update current Encoding modes of all lanes for this Slice
    enable_bit = 1 if enable == 'en' else 0
   
   
    
    LT_REG_BASE        = [0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,   0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F]
    TX_B_SIDE_REG_BASE = [0x80,0x82,0x84,0x86,0x88,0x8A,0x8C,0x8E,   0x80,0x82,0x84,0x86,0x88,0x8A,0x8C,0x8E]
    RX_B_SIDE_REG_BASE = [0x81,0x83,0x85,0x87,0x89,0x8B,0x8D,0x8F,   0x81,0x83,0x85,0x87,0x89,0x8B,0x8D,0x8F]
    RX_A_SIDE_REG_BASE = [0x70,0x72,0x78,0x7A,  0x70,0x72,0x78,0x7A, 0x70,0x72,0x78,0x7A,  0x70,0x72,0x78,0x7A]


    if print_en: print("\nfec_monitor_setup: Lanes Initialized for Gearbox"), 
    
    for ln in lanes:    
        get_lane_mode(ln)
        ####disable link training of this lane
        wreg(0x3000+(0x100*ln),0x0000,lane=0)
        ####disable AutoNeg of this lane
        wreg(0xe000+(0x100*ln),0xc000,lane=0)   
            
        wreg([0x0f3,[14,8]], 0x00      , ln) ## [15]->1, TX phase rotator value
        wreg([0x0f3,  [15]], enable_bit, ln) ## [15]->1, TX phase rotator enable
       #wreg([0x0f3,   [2]], enable_bit, ln) # 0 or 1 for pam4
        wreg([0x0f3,   [2]],          0, ln) # TRF pol is controlled by register 0x79 or 0x17a

        if enable_bit==1: # loopback_mode enabled, TX in functional mode
            prbs_mode_select(lane=ln, prbs_mode='functional') # Put TX in Functional Mode
        else:                         # loopback_mode disabled, TX in PRBS mode
            prbs_mode_select(lane=ln, prbs_mode='prbs')    # Put TX in PRBS Mode7
        
        if gEncodingMode[gSlice][ln][0] == 'pam4':
            if enable_bit==1: # Lane is PAM4 
                 wreg(0x079,0x2474,ln)  ## PAM4 TRF 53G
            else:
                 wreg(0x079,0x00a4,ln)  ## PAM4 TRF, set to the same value as in the init_pam4/init_nrz
        else: # Lane is in NRZ
            if enable_bit==1: 
                wreg(0x17a,0x116C,ln) ##  NRZ TRF, 25G
            else:
                wreg(0x17a,0x00a4,ln) ##  NRZ TRF, set to the same value as in the init_pam4/init_nrz
        
        wreg((TX_B_SIDE_REG_BASE[ln]*0x100)+0xf1, 0x000f) # NRZ en_40t_clk_rx B
        wreg((TX_B_SIDE_REG_BASE[ln]*0x100)+0xfb, 0x7fb7) # NRZ en_40t_clk_tx B

        wreg((RX_A_SIDE_REG_BASE[ln]*0x100)+0xf1, 0x000f) # PAM4 en_40t_clk_rx A 
        wreg((RX_A_SIDE_REG_BASE[ln]*0x100)+0xfb, 0x7fb7) # PAM4 en_40t_clk_tx A
        
        #### OVERWRITE some parameters here with newer recommended values
        # wreg((TX_B_SIDE_REG_BASE[ln]*0x100)+0xf3, 0x8004) # NRZ TX reg_phase_rotator_flip_tx[2]
        # wreg((RX_B_SIDE_REG_BASE[ln]*0x100)+0x7a, 0x116c) # 0x116C) # NRZ TRF setting
        
        # wreg((RX_A_SIDE_REG_BASE[ln]*0x100)+0xf3, 0x8000) # PAM4 phase rotator
        # wreg((RX_A_SIDE_REG_BASE[ln]*0x100)+0x79, 0x2494) # 3894) # PAM4 TRF setting
    

    if A_lanes==[0] and B_lanes==[8,9]: # 50G mode with BH
        if print_en: print("\nfec_monitor_setup: Gearbox 50G Lanes A0 and B0/B1"), 
        ###top_tx_pi_sel###
        wreg(0x98d1, 0x0000)  # bit[15:12]: B0_Rx -> A7_Tx
        wreg(0x98d2, 0x0000)  # B0-> A0
        wreg(0x98d3, 0x0000)  #laneB7-B4
        wreg(0x98d4, 0x1100)  # A0->B1  and A0 ->B0

    if print_en: print("Done!"), 

    if (0): # original setup with Spirent box
        ###top_tx_pi_sel###
        wreg(0x98d1, 0x0000)  # bit[15:12]: B0_Rx -> A7_Tx
        wreg(0x98d2, 0x0000)  # B0-> A0
        wreg(0x98d3, 0x0000)  #laneB7-B4
        wreg(0x98d4, 0x1100)  # A0->B1  and A0 ->B0
        
    elif A_lanes==[1] and B_lanes==[10,11]: # 50G mode with BH
        if print_en: print("\nfec_monitor_setup: Gearbox 50G Lanes A1 and B2/B3"), 
        ###top_tx_pi_sel###
        wreg(0x98d1, 0x0000)  # bit[15:12]: B0_Rx -> A7_Tx
        wreg(0x98d2, 0x0020)  # B2-> A1
        wreg(0x98d3, 0x0000)  #laneB7-B4
        wreg(0x98d4, 0x1100)  # A1->B2  and A1 ->B3

    # FINALLY, set Refclk source of TX_PLL of all lanes to either RXRecovered or the Crystal
    for ln in lanes:
        wreg([0x0f4,   [0]], enable_bit, ln) ## TX_PLL refclk Source 0: crystal 1:RX_PLL recovered clock

####################################################################################################  
# Set PAM4-NRZ Lanes in Gear Box mode, A-side PAM4, B-side: NRZ
#
# Examples:
# A_lanes=[0], B_lanes=[ 8, 9]: 1x50G PAM4 A0 <-> 2x25G NRZ  B1-B0
# A_lanes=[1], B_lanes=[10,11]: 1x50G PAM4 A1 <-> 2x25G NRZ  B2-B3
#
####################################################################################################
def fec_monitor_clear(A_lanes=[0], B_lanes=[8,9], print_en=1):

    lanes = [A_lanes[0], B_lanes[0]-8] # only need to know the first lane on each side
    #get_lane_mode(lanes) # update current Encoding modes of all lanes for this Slice
   
    A_SIDE_FEC_REG_BASE = [0x48,0x49,0x4A,0x4B,0x48,0x49,0x4A,0x4B,]
    B_SIDE_FEC_REG_BASE = [0x58,0x58,0x59,0x59,0x5A,0x5A,0x5B,0x5B]
    
    if print_en: print("\fec_monitor_clear: Setting up FEC for Gearbox mode"), 
    

    # expecting FEC lock condition here: 0x40c9=0x4303 or 0xc303
    for ln in lanes:
        get_lane_mode(ln)
        #if A_lanes==[0] and B_lanes==[8,9]: # 50G mode with BH
        if print_en: print("\nGearbox 50G: Initalizing FECs on Lanes A0 and B0/B1..."), 

        ####FEC####
        wreg(0x9858, 0x0008) #0101)   # [3]=1, FEC A0 enabled (50G-1 mode)
        #wreg(0x9857, 0x0202) #0101)   #fec_en on PAM4 side only, 50G-1 lane
        wreg(0x9857, 0x0111) #0101)   #fec_en on both sides, 50G-1 lane, 1.    9857 should be 0111

        wreg((A_SIDE_FEC_REG_BASE[ln]*0x100)+0x80, 0x00c0); # Reset FEC A Registers 
        wreg((A_SIDE_FEC_REG_BASE[ln]*0x100)+0x80, 0x00c0); # Reset FEC B Registers 
        time.sleep(.1)
        
                
        ###need set it manually
        wreg(0x4850, 0x1003); 
        wreg(0x5850, 0x1003); 

        wreg(0x4880, 0x0f00); # Reset FEC A TX and RX
        wreg(0x5880, 0x0f00); # Reset FEC B TX and RX
        time.sleep(.5)
        wreg(0x4880, 0x0e00); # 3.    Wr 4880=0e00 to release A side FEC Rx, 40c9 should go to 4303, indicating A side FEC Rx has locked to 50G traffic from BH
        time.sleep(2)
        wreg(0x5880, 0x0b00); # 5.    Wr 5880=0b00 to release reset for b side FEC tx, b side B0 B1 rx eye should be good
        time.sleep(2)
        
        # 6. If everything is good(clocks are good due to TRF settings), 
        # register 589c should be toggling in a narrow range, 
        # 589d should be a non-zero value and 589e should be a value < 0x0500.
        
        wreg(0x5880, 0x0a00); # 7.    If 6 is OK, wr 5880=0a00 to release reset for b side FEC rx, reg 50c9 should go to 4303 to indicate B side FEC Rx locks
        time.sleep(2)
        wreg(0x4880, 0x0a00); # 8.    If 7 is OK, wr 4880=0a00 to release A side FEC tx, check registers 489c/d/e, they should have similar behavior as 589c/d/e
        time.sleep(.1)

        #wreg(0x4880, 0x0000)
        #9.    If above is OK, we should be sending 50G traffic back to BH
        # expecting FEC lock condition here: 0x40c9=0x4303 or 0xc303
    
#    elif A_lanes==[1] and B_lanes==[10,11]: # 50G mode with BH
        if print_en: print("\nGearbox 50G: Initalizing FECs on Lanes A1 and B2//B3..."), 
        
        wreg(0x4980, 0x00c0); # Reset FEC A Registers 
        wreg(0x5980, 0x00c0); # Reset FEC B Registers 
        time.sleep(.1)
        
        ####FEC####
        wreg(0x9858, 0x0008) #0101)   #fec_en, 50G-1 lane
        #wreg(0x9857, 0x0202) #0101)   #fec_en on PAM4 side only, 50G-1 lane
        wreg(0x9857, 0x0222) #0101)   #fec_en on both sides, 50G-1 lane, 1.    9857 should be 0111
                
        ###need set it manually
        wreg(0x4950, 0x1003); 
        wreg(0x5950, 0x1003); 

        wreg(0x4980, 0x0f00); # Reset FEC A TX and RX
        wreg(0x5980, 0x0f00); # Reset FEC B TX and RX
        time.sleep(.5)
        wreg(0x4980, 0x0e00); # 3.    Wr 4880=0e00 to release A side FEC Rx, 40c9 should go to 4303, indicating A side FEC Rx has locked to 50G traffic from BH
        time.sleep(2)
        wreg(0x5980, 0x0b00); # 5.    Wr 5880=0b00 to release reset for b side FEC tx, b side B0 B1 rx eye should be good
        time.sleep(2)
        
        # 6. If everything is good(clocks are good due to TRF settings), 
        # register 589c should be toggling in a narrow range, 
        # 589d should be a non-zero value and 589e should be a value < 0x0500.
        
        wreg(0x5980, 0x0a00); # 7.    If 6 is OK, wr 5880=0a00 to release reset for b side FEC rx, reg 50c9 should go to 4303 to indicate B side FEC Rx locks
        time.sleep(2)
        wreg(0x4980, 0x0a00); # 8.    If 7 is OK, wr 4880=0a00 to release A side FEC tx, check registers 489c/d/e, they should have similar behavior as 589c/d/e
        time.sleep(.1)

        #wreg(0x4880, 0x0000)
        #9.    If above is OK, we should be sending 50G traffic back to BH
        # expecting FEC lock condition here: 0x40c9=0x4303 or 0xc303
    if print_en: print("Done!"),      
   
####################################################################################################
# 
# Configure specified lane(s) in NRZ or PAM4 mode
#
# Note: this function sets minimum requirement for a lane to be in PAM4 or NRZ mode
#       It does not do a complete initialization of the lane in a NRZ/PAM4 mode
#       See init_lane() for complete initialization.
####################################################################################################
def set_lane_mode(mode='pam4',lane=None):

    lanes = get_lane_list(lane)

    for ln in lanes:
        if mode.upper()!='PAM4': # put lanes in NRZ mode
            wreg([0x041,[15]],0, ln) # Disable PAM4 mode
            wreg([0x0a0,[13]],0, ln) # Disable PAM4 PRBS Generator
            wreg([0x0b0, [1]],1, ln) # Enable NRZ mode
            wreg([0x0b0,[11]],1, ln) # Enable NRZ PRBS Generator
        else: #################### put lanes in PAM4 mode
            wreg([0x0b0, [1]],0, ln) # Disable NRZ mode
            wreg([0x0b0,[11]],0, ln) # Disable NRZ PRBS Generator
            wreg([0x041,[15]],1, ln) # Enable PAM4 mode
            wreg([0x0a0,[13]],1, ln) # Enable PAM4 PRBS Generator    
####################################################################################################
# 
# Checks to see if a lane is in PAM4 or NRZ mode
#
# updates the global gEncodingMode variable for this Slice and this lane
# returns: list of [NRZ/PAM4, speed] per lane
#
####################################################################################################
def get_lane_mode(lane=None):
    
    lanes = get_lane_list(lane)
 
    global gEncodingMode
    
    for ln in lanes:

        if rreg([0x0ff,[12]],ln)==0 or rreg([0x1ff,[12]],ln)==0: # Lane's bandgap is OFF
            #print ("\n Slice %d lane %2d is OFF"%(gSlice,ln)),
            data_rate= 1.0
            gEncodingMode[gSlice][ln] = ['off',data_rate]
            lane_mode_list[ln] = 'off'

        elif rreg([0xb0,[1]],ln) == 0 and rreg([0x41,[15]],ln) == 1:
            #print ("\n Slice %d lane %2d is PAM4"%(gSlice,ln)),
            data_rate= get_lane_pll(ln)[ln][0][0]
            gEncodingMode[gSlice][ln] = ['pam4',data_rate]
            lane_mode_list[ln] = 'pam4'
        else:
            #print ("\n Slice %d lane %2d is NRZ"%(gSlice,ln)),
            data_rate= get_lane_pll(ln)[ln][0][0]
            gEncodingMode[gSlice][ln] = ['nrz',data_rate]
            lane_mode_list[ln] = 'nrz'
            
    return gEncodingMode
####################################################################################################
# 
# Initialize lane(s) in NRZ or PAM4 mode
#
# Note: this function does a complete initialization of the lane in a NRZ/PAM4 mode
#       It does not adapt the RX of the lane to the connected channel
#       See opt_lane() for optimization of lane
####################################################################################################
def init_lane(mode='pam4',datarate=None,input_mode='ac',lane=None):

    lanes = get_lane_list(lane)

    for ln in lanes:
        if 'NRZ' in mode.upper(): # put lanes in NRZ mode
            if datarate!=None:  init_lane_nrz (datarate,input_mode,ln) # NRZ at exactly the requested datarate
            elif '10' in mode:  init_lane_nrz (10.3125, input_mode,ln) # NRZ-10G
            elif '20' in mode:  init_lane_nrz (20.6250, input_mode,ln) # NRZ-20G
            elif '25' in mode:  init_lane_nrz (25.78125,input_mode,ln) # NRZ-25G
            else:               init_lane_nrz (25.78125,input_mode,ln) # NRZ-25G, or exactly the requested datarate      
        else: #################### put lanes in PAM4 mode
            init_lane_pam4(datarate,input_mode,ln)  
####################################################################################################
# 
# Initialize and adapt lane(s) in NRZ or PAM4 mode
#
# Note: this function does a complete initialization and adaptation of lane in a NRZ/PAM4 mode
#       It optimizes the RX of the lane to the connected channel
####################################################################################################
def opt_lane(mode='pam4',datarate=None, input_mode='ac',lane=None):

    lanes = get_lane_list(lane)
    set_bandgap('on', 'all') 
    set_top_pll(pll_side='both', freq=195.3125)
    
    #### FW-based Adaptation
    if fw_loaded(print_en=0):
        for ln in lanes:
            #init_lane_for_fw(mode = mode,tx_pol=1, rx_pol=RxPolarityMap[gSlice][ln],input_mode = input_mode,lane=ln)
            init_lane(mode,datarate,input_mode,ln)
        fw_config_lane(mode,datarate,lanes)
        fw_adapt_wait (lane=lanes, print_en=2)
        
    #### Python-based Adaptation
    else:
        for ln in lanes:
            if 'NRZ' in mode.upper(): # put lanes in NRZ mode and optimize them
                if datarate!=None:  opt_lane_nrz (datarate,input_mode,ln) # NRZ-25G, or exactly the requested datarate
                elif '10' in mode:  opt_lane_nrz (10.3125, input_mode,ln) # NRZ-10G
                elif '20' in mode:  opt_lane_nrz (20.6250, input_mode,ln) # NRZ-20G
                elif '25' in mode:  opt_lane_nrz (25.78125,input_mode,ln) # NRZ-25G
                else:               opt_lane_nrz (25.78125,input_mode,ln) # NRZ-25G, or exactly the requested datarate
            else: #################### put lanes in PAM4 mode and optimize them
                opt_lane_pam4(datarate,input_mode,ln)  
            
####################################################################################################
# 
# Initialize target lane(s) in NRZ mode
#
# Note: this function does a complete initialization of the lane in a NRZ mode
#       It does not adapt the RX of the lane to the connected channel
#       See opt_lane() for complete initialization.
####################################################################################################
def init_lane_for_fw(mode='nrz',tx_pol=1,rx_pol=0, input_mode='ac',lane=None):
    #print'...init_lane_for_fw'
    lanes = get_lane_list(lane)
    global gEncodingMode


    for ln in lanes:
        set_lane_mode(mode=mode,lane=ln)
        ####################### put lane in PAM4 mode
        if ('PAM4' in mode.upper()):
            wreg([0x0af,[5,1]],0x4,lane=LanePartnerMap[gSlice][ln][1]) # TX Taps scales = (0.5,0.5, 1, 0.5,0.5)
            tx_taps(2,-8,17,0,0,lane=ln)
            
            # PAM4 mode, gc=en | Pre=dis | masblsb= default
            gc(1,1,lane=ln)
            pc(0,0,lane=ln)
            msblsb(0,0,lane=ln)
            
            if gPrbsEn:
                prbs_mode_select(lane=ln,prbs_mode='prbs') # PAM4 mode, tx pam4 prbs31
                #wreg(0x042,0xb3fd,ln) 
            else:
                prbs_mode_select(lane=ln,prbs_mode='functional') # PAM4 mode, TX pam4 FUNCTIONAL MODE
                #wreg(0x042,0xb3fd,ln) # PAM4 mode, Rx PAM4 gc=en | Pre=dis

        ####################### put lane in NRZ mode
        if ('NRZ' in mode.upper()):
            wreg([0x0af,[5,1]],0x4,lane=LanePartnerMap[gSlice][ln][1]) # TX Taps scales = (0.5,0.5, 1, 0.5,0.5)
            tx_taps(0,-8,17,0,0,lane=ln)
            
            # NRZ mode, gc=dis | Pre=dis | masblsb= default (none applies in NRZ mode)
            gc(0,0,lane=ln)
            pc(0,0,lane=ln)
            msblsb(0,0,lane=ln)
            #wreg([0x161,[8]],1,ln) # NRZ freq_loop enable ## FW set this during init_lane
            if gPrbsEn: #print'PRBS_ON'
                prbs_mode_select(lane=ln,prbs_mode='prbs')
                #wreg(0x161,0x7520,ln) # NRZ mode, Rx checker
            else:      #print'FUNCTIONAL_ON'
                prbs_mode_select(lane=ln,prbs_mode='functional')
                #wreg(0x161,0x3520,ln) # NRZ mode, Rx checker off
          
        if chip_rev()==2.0:
            wreg([0x07b,[3]],  1, ln) # R2.0 PAM4 BLWC Pol = 1
            wreg([0x17c,[15]], 0, ln) # R2.0 NRZ BLWC en = 0 (disabled)
                     
        if (input_mode=='dc'):
            wreg(c.rx_en_vcominbuf_addr,0,ln) # DC-Coupled mode 0x1E7[11]=0
        else:
            wreg(c.rx_en_vcominbuf_addr,1,ln) # AC-Coupled mode 0x1E7[11]=1

        # Set the polarities of the lanes
        pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
        

def init_lane_nrz(datarate=None, input_mode='ac',lane=None):

    lanes = get_lane_list(lane)
    global gEncodingMode
    c=NrzReg
    
    if datarate==None:  datarate=25.78125
    # if datarate!=None:  datarate=datarate # NRZ at exactly the requested datarate
    # elif '10' in mode:  datarate=10.3125  # NRZ-10G
    # elif '20' in mode:  datarate=20.6250  # NRZ-20G
    # elif '25' in mode:  datarate=25.78125 # NRZ-25G
    # else:               datarate=25.78125 # NRZ-25G   

    for ln in lanes:
        #print("\ninit_lane_nrz(): Lane %s, Target DataRate=%3.5fG, Actual DataRate=%3.5f G"%(lane_name_list[ln],datarate,get_lane_pll(ln)[ln][0][0])),
        gEncodingMode[gSlice][ln] = ['nrz',datarate]
        lane_mode_list[ln] = 'nrz'
        ####disable AN/LT registers as default
        wreg(0x3000+(0x100*ln),0x0000,lane=0)
        wreg(0xe000+(0x100*ln),0xc000,lane=0)            
        # ---------------- LANE Q NRZ init begin

        #wreg(c.rx_lane_rst_addr, 1, ln) # Keep lane in Reset while programming lane
        #wreg(c.rx_pll_pu_addr,   0, ln) # Power down RX PLL while prgramming PLL
        #wreg(c.tx_pll_pu_addr,   0, ln) # Power down RX PLL while prgramming PLL
        ####################### NRZ mode, Per-Lane PLLs ###
        wreg(0x0D7,0x0000,ln) # NRZ mode, TX_PLL, Fractional PLL off  
        wreg(0x0D8,0x0000,ln) # NRZ mode, TX_PLL, Fractional PLL off  
        wreg(0x1F0,0x0000,ln) # NRZ mode, RX_PLL, Fractional PLL off  
        wreg(0x1F1,0x0000,ln) # NRZ mode, RX_PLL, Fractional PLL off    
        
       #wreg(0x0FF,0x5df6,ln) # NRZ mode, TX_BG bandgap | div4=0
       #wreg(0x0FE,0x213e,ln) # NRZ mode, TX PLLN = 66*195.3125 | [3:1] //vcoi=0x7
        wreg(0x0FF,0x5df4,ln) # NRZ mode, TX_BG bandgap | div4=0, div2_pass=0
        wreg(0x0FE,0x10bf,ln) # NRZ mode, TX PLLN = 33*195.3125 | [3:1] //vcoi=0x7
        wreg(0x0FD,0x5636,ln) # NRZ mode, TX PLL,bypass_pll=0
       #wreg(0x0FD,0x5436,ln) # NRZ mode, TX PLL, clear bypass 1p7 regulator 0xFD[9]
        wreg(0x0FC,0x7236,ln) # NRZ mode, TX PLL,vvco_reg
        wreg(0x0FB,0x7fb7,ln) # NRZ mode, TX PLL,vref_intp
        wreg(0x0FA,0x8010,ln) # NRZ mode, TX PLL,testmode_tx_pll
        wreg(0x0DB,0x2100,ln) # NRZ mode, TX PLL,[14:8] // tx_vcocap=rx_vcocap=33 or 34 for fvco=25.78125G
        wreg(0x0DA,0x7de6,ln) # NRZ mode, TX PLL,[14:10]// bm_vco=0x1f
        wreg(0x0D9,0xb760,ln) # NRZ mode,
                  
        wreg(0x1FF,0x5db1,ln) # NRZ mode, RX_BG bandgap | [15:13]//vbg=0
      
        wreg(0x1FD,0x213f,ln) # NRZ mode, RX_PLLN= 33*2*195.3125 |  [3:1]  //vcoi=0x7
       #wreg(0x1FC,0x1048,ln) # NRZ mode, RX PLL, clear bypass 1p7 regulator 0x1FC[9]
        wreg(0x1F5,0x4340,ln) # NRZ mode, RX PLL, lcvco_cap [15:9] //rx_vcocap=33 or 34 for fvco=25.78125G
        wreg(0x1F4,0x7de6,ln) # NRZ mode, RX PLL, en_div2=0 | [14:10]//bm_vco=0x1f
        wreg(0x1F3,0xb760,ln) # NRZ mode, RX PLL, pu_intp_rx



        ####################### analog ###

        wreg(0x0f1,0x000f,ln) # NRZ mode, pu_adc|pu_clkcomp|pu_clkreg
        wreg(0x0ef,0x9230,ln) # NRZ mode, vrefadc
        wreg(0x0ee,0x691c,ln) # NRZ mode, calnbs_top|bot
        wreg(0x0ed,0x7777,ln) # NRZ mode, edge
        wreg(0x0eb,0x67fc,ln) # NRZ mode, TX Swing 
                    
        wreg(0x1f9,0x7686,ln) # NRZ mode, vrefagcdegen
        wreg(0x1f6,0x71b0,ln) # NRZ mode, vagcbufdac
        wreg(0x1e7,0xc34c,ln) # NRZ mode, pu_agc|pu_agcdl | en_vcominbuf=0
        wreg(0x1e6,0x88a5,ln) # NRZ mode, bypass_agcreg
        wreg(0x1e5,0x6c38,ln) # NRZ mode, 4e3c # vrefagc1p5reg
                  
        wreg(0x1dd,0xc162,ln) # NRZ mode, c1e2 # ffe_delay | en_skef | skef_value
        wreg(0x1da,0x6f00,ln) # NRZ mode, 6d80 # clkphase0
                  
        wreg(0x1d4,0x01c0,ln) # NRZ mode, agcgain1 | agacgain2
        wreg(0x1d5,0x0100,ln) # NRZ mode, blwc_en
        wreg(0x1d6,0xee00,ln) # NRZ mode, en_az_short_pulse

        #######################  NRZ Mode Configuration
        wreg(0x041,0x0000,ln) # NRZ mode, Toggle PAM4_RX_ENABLE
        wreg(0x041,0x8000,ln) # NRZ mode, Toggle PAM4_RX_ENABLE
        wreg(0x041,0x0000,ln) # NRZ mode, Toggle PAM4_RX_ENABLE
        wreg(0x0b0,0x4802,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode (4802) | Toggle PAM4_TX_ENABLE
        wreg(0x0b0,0x0000,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode (4802) | Toggle PAM4_TX_ENABLE
        wreg(0x0b0,0x4802,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode (4802) | Toggle PAM4_TX_ENABLE
        wreg(0x1e7,0x836c,ln) # NRZ mode, ffe disable in nrz mode (agc_dl=0)

        #######################  NRZ SM Configuration
        #wreg(0x101,0x4201,ln)
        wreg(0x101,0x0201,ln) # NRZ mode, delta_freeze  
        wreg(0x102,0x1006,ln) # NRZ mode, cntr_target=0x100 during link-up, change to 0x002 after
        wreg(0x103,0x5933,ln)
        wreg(0x104,0x700a,ln)
       #wreg(0x105,0x8682,ln)  #8e82, # UPDATE 20180521
       #wreg(0x105,0x8e82,ln)  #8e82, 
        wreg(0x106,0xA100,ln) 
        wreg(0x108,0x688B,ln)  
        wreg(0x10C,0x8000,ln)  
        wreg(0x13B,0xe000,ln) # NRZ mode, Set KP=6, # UPDATE 20180521
        #wreg(0x145,0x8080,ln) # commented out CHANGE_FOR_10G, # UPDATE 20180521
        wreg(0x14D,0x8000,ln) # NRZ mode, OW INIT_FREQ=0  
        wreg([0x014D, [10,0]], 0x7D8, ln) #set one initial freq to be -160ppm, # CHANGE_FOR_10G, # UPDATE 20180521
        wreg(0x14F,0x0100,ln)
        wreg(0x150,0x5040,ln)
        wreg(0x161,0x0120,ln) # NRZ mode, Rx checker power-down.
        wreg(0x163,0x0080,ln)
        wreg(0x164,0x8080,ln)
        wreg(0x179,0x8874,ln) # 887e bb_mode=1
        wreg(0x180,0x3500,ln)  
        wreg(0x181,0x1000,ln)  
        wreg([0x0181, [10,0]], 0x7D8, ln) #set one initial freq to be -160ppm, # CHANGE_FOR_10G, # UPDATE 20180521
        wreg(0x182,0xD800,ln)  
                
        #wreg(0x18C,0x0040,ln) # CHANGE_FOR_10G # UPDATE 20180521         
        wreg(0x176,0x418A,ln) #209a ctle_map
        wreg(0x177,0x3837,ln) 
        wreg(0x178,0x97EE,ln)
        wreg(0x14E,0x3800,ln) #CTLE=7

        wreg(0x17C,0x0000,ln) # REV_2.0 default  = 0x8001
        
        ####################### NRZ mode, TX setting
        if gNrzTxSourceIsCredo:
            wreg(0x0a5,0x0000,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode, pre2
            wreg(0x0a7,0xf800,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode, pre1
            wreg(0x0a9,0x1100,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode, main
            wreg(0x0ab,0x0000,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode, post1
            wreg(0x0ad,0x0000,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode, post2
            wreg(0x0af,0xfc08,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode, tx_automode|no gc|nopre |SHmode

        ####################### NRZ mode,  TX PRBS gen and Rx PRBS checker
        if gPrbsEn:
            wreg(0x0a0,0xeb20,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode, tx prbs31
            wreg(0x0a0,0xe320,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode, tx prbs31
            wreg(0x0a0,0xeb20,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode, tx prbs31
            wreg(0x161,0x7520,ln) # NRZ mode, Rx checker
        else:
            wreg(0x0a0,0x0120,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode, TX functional mode
            wreg(0x0a0,0x0120,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode, TX functional mode
            wreg(0x0a0,0x0120,lane=LanePartnerMap[gSlice][ln][1]) # NRZ mode, TX functional mode
            wreg(0x161,0x3520,ln) # NRZ mode, Rx checker off
        
       #set_lane_pll(tgt_pll='both',datarate=datarate, div2=0, lane=ln)
        
        if chip_rev()==2.0:
            wreg([0x07b,[3]],  1, ln) # R2.0 PAM4 BLWC Pol = 1
            wreg([0x17c,[15]], 0, ln) # R2.0 NRZ BLWC en = 0 (disabled)
            if datarate < 24.0: # If 10G or 20G using Frac-N, expanded to 20 bits for TX-PLL 
                wreg([0x0d9,[3,0]],0x6, ln) # NRZ-10G/20G mode, 0x0D9[3:0] TX PLL FRAC_N[19:16]=0x6 = 0.4 (R2.0: 20 bits, R1.0: 16 bits)

        if datarate < 15.0: # NRZ 10.3125G , NRZ Half-Rate Mode
            #### disable Fractional PLLs while programming PLLs,  # UPDATE 20180521
            wreg(c.tx_pll_frac_en_addr,     0, ln) # NRZ-10G mode, 0x0D7   [13] TX PLL FRAC_EN=0 while programming PLLs
            wreg(c.rx_pll_frac_en_addr,     0, ln) # NRZ-10G mode, 0x1F0   [13] RX PLL FRAC_EN=0 while programming PLLs
            
            wreg(c.tx_pll_lvcocap_addr,    85, ln) # NRZ-10G mode, 0x0DB [14:8] TX PLL VCOCAP=84 or 85 for fvco=20.625G                
            wreg(c.tx_pll_n_addr,          26, ln) # NRZ-10G mode, 0x0FE [15:7] TX_PLL N= 26.4*2*195.3125
            wreg(c.tx_pll_div4_addr,        0, ln) # NRZ-10G mode, 0x0FF    [0] TX_PLL DIV4=0
            wreg(c.tx_pll_div2_addr,        0, ln) # NRZ-10G mode, 0x0FF    [1] TX PLL DIV2=0
            if chip_rev()==2.0:
                wreg([0x0d9,[3,0]],          0x6, ln) # NRZ-10G mode, 0x0D9  [3:0] TX PLL FRAC_N[19:16]=0x6    = 0.4 (R2.0: 20 bits, R1.0: 16 bits)
                wreg(c.tx_pll_frac_n_addr, 0x6666, ln) # NRZ-10G mode, 0x0D8 [15:0] TX PLL FRAC_N[15: 0]=0x6666 = 0.4
            else:
                wreg([0x0d9,[3,0]],          0x0, ln) # NRZ-10G mode, 0x0D9  [3:0] TX PLL FRAC_N[19:16]=0x0    = 0.4 (R2.0: 20 bits, R1.0: 16 bits)
                wreg(c.tx_pll_frac_n_addr, 0x6666, ln) # NRZ-10G mode, 0x0D8 [15:0] TX PLL FRAC_N[15: 0]=0x6666 = 0.4
            
            wreg(c.rx_pll_frac_order_addr,  2, ln) # NRZ-10G mode, 0x1F0[15:14] RX PLL FRAC_ORDER =10
            
            wreg(c.rx_pll_lvcocap_addr,    85, ln) # NRZ-10G mode, 0x1F5 [15:9] RX PLL VCOCAP=84 or 85 for fvco=20.625G                
            wreg(c.rx_pll_n_addr,          52, ln) # NRZ-10G mode, 0x1FD [15:7] RX_PLL N= 52.8*1*195.3125
            wreg(c.rx_pll_div4_addr,        0, ln) # NRZ-10G mode, 0x1FF    [6] RX_PLL DIV4=0
            wreg(c.rx_pll_div2_addr,        1, ln) # NRZ-10G mode, 0x1F4    [8] RX PLL DIV2=1
            wreg(c.rx_pll_frac_n_addr, 0xCCCC, ln) # NRZ-10G mode, 0x1F1 [15:0] RX PLL FRAC_N=0xCCCC = 0.8
            wreg(c.tx_pll_frac_order_addr,  2, ln) # NRZ-10G mode, 0x0D7[15:14] TX PLL FRAC_ORDER =10

            wreg(c.rx_pll_pu_addr,          1, ln) # NRZ-10G mode,Power up RX PLL after prgramming PLL and before toggling FRAC_EN
            wreg(c.tx_pll_pu_addr,          1, ln) # NRZ-10G mode,Power up TX PLL after prgramming PLL and before toggling FRAC_EN

            #### Enable Fractional PLLs after programming PLLs, # UPDATE 20180521
            wreg(c.tx_pll_frac_en_addr,     1, ln) # NRZ-10G mode, 0x0D7   [13] TX PLL FRAC_EN=1 after programming PLLs
            wreg(c.rx_pll_frac_en_addr,     1, ln) # NRZ-10G mode, 0x1F0   [13] RX PLL FRAC_EN=1 after programming PLLs               
                                                        
            wreg(c.tx_mode10g_en_addr,      1, ln) # NRZ-10G mode, TX_NRZ_10G_EN=1 (or Enable NRZ Half-Rate Mode)
            wreg(c.rx_mode10g_addr,         1, ln) # NRZ-10G mode, RX_NRZ_10G_EN=1 (or Enable NRZ Half-Rate Mode)
                                                        
            wreg(c.rx_delta_adapt_en_addr,  0, ln) # NRZ-10G mode, Disable Delta Adaptation loop(0x0101=0x0201)
            wreg(0x0165,0x0001, ln)                # NRZ-10G Mode, INTP Half Rate mode [0] = 1
            wreg(0x0175,0xC6AF, ln)                # NRZ-10G Mode, Margin Counter Phases 4321 en [7:4]=1010
            wreg(0x0100,0x4010, ln)                # NRZ-10G mode, F1F2_INIT_0, 0x5185
            wreg(0x0107,0x4013, ln)                # NRZ-10G mode, F1F2_INIT_1, 0x5185
            wreg(0x0183,0x4013, ln)                # NRZ-10G mode, F1F2_INIT_2, 0x5185
            wreg(0x0184,0x8007, ln)                # NRZ-10G mode, F1F2_INIT_3, 0x9185
            wreg(0x0185,0x4013, ln)                # NRZ-10G mode, F1F2_INIT_4, 0x5185
            wreg(0x0186,0xC014, ln)                # NRZ-10G mode, F1F2_INIT_5, 0xD185
            wreg(0x0187,0xC012, ln)                # NRZ-10G mode, F1F2_INIT_6, 0x5185
            wreg(0x0188,0x4006, ln)                # NRZ-10G mode, F1F2_INIT_7, 0x5185
            
        elif datarate < 24.0: # NRZ 20.6250G, , NRZ Full-Rate Mode
            #### disable Fractional PLLs while programming PLLs,# UPDATE 20180521
            wreg(c.tx_pll_frac_en_addr,     0, ln) # NRZ-20G mode, 0x0D7   [13] TX PLL FRAC_EN=0 while programming PLLs
            wreg(c.rx_pll_frac_en_addr,     0, ln) # NRZ-20G mode, 0x1F0   [13] RX PLL FRAC_EN=0 while programming PLLs
        
            wreg(c.tx_pll_lvcocap_addr,    85, ln) # NRZ-20G mode, 0x0DB [14:8] TX PLL VCOCAP=84 or 85 for fvco=20.625G                
            wreg(c.tx_pll_n_addr,          26, ln) # NRZ-20G mode, 0x0FE [15:7] TX_PLL N= 26.4*2*195.3125
            wreg(c.tx_pll_div4_addr,        0, ln) # NRZ-20G mode, 0x0FF    [0] TX_PLL DIV4=0
            wreg(c.tx_pll_div2_addr,        0, ln) # NRZ-20G mode, 0x0FF    [1] TX PLL DIV2=0
            if chip_rev()==2.0:
                wreg([0x0d9,[3,0]],          0x6, ln) # NRZ-10G mode, 0x0D9  [3:0] TX PLL FRAC_N[19:16]=0x6    = 0.4 (R2.0: 20 bits, R1.0: 16 bits)
                wreg(c.tx_pll_frac_n_addr, 0x6666, ln) # NRZ-10G mode, 0x0D8 [15:0] TX PLL FRAC_N[15: 0]=0x6666 = 0.4
            else:
                wreg([0x0d9,[3,0]],          0x0, ln) # NRZ-10G mode, 0x0D9  [3:0] TX PLL FRAC_N[19:16]=0x0    = 0.4 (R2.0: 20 bits, R1.0: 16 bits)
                wreg(c.tx_pll_frac_n_addr, 0x6666, ln) # NRZ-10G mode, 0x0D8 [15:0] TX PLL FRAC_N[15: 0]=0x6666 = 0.4
            wreg(c.tx_pll_frac_order_addr,  2, ln) # NRZ-20G mode, 0x0D7[15:14] TX PLL FRAC_ORDER =10
                                                        
            wreg(c.rx_pll_lvcocap_addr,    85, ln) # NRZ-20G mode, 0x1F5 [15:9] RX PLL VCOCAP=84 or 85 for fvco=20.625G                
            wreg(c.rx_pll_n_addr,          52, ln) # NRZ-20G mode, 0x1FD [15:7] RX_PLL N= 52.8*1*195.3125
            wreg(c.rx_pll_div4_addr,        0, ln) # NRZ-20G mode, 0x1FF    [6] RX_PLL DIV4=0
            wreg(c.rx_pll_div2_addr,        1, ln) # NRZ-20G mode, 0x1F4    [8] RX PLL DIV2=1
            wreg(c.rx_pll_frac_n_addr, 0xCCCC, ln) # NRZ-20G mode, 0x1F1 [15:0] RX PLL FRAC_N=0xCCCC = 0.8
            wreg(c.rx_pll_frac_order_addr,  2, ln) # NRZ-20G mode, 0x1F0[15:14] RX PLL FRAC_ORDER =10
            
            wreg(c.rx_pll_pu_addr,          1, ln) # NRZ-20G mode,Power up RX PLL after prgramming PLL and before toggling FRAC_EN
            wreg(c.tx_pll_pu_addr,          1, ln) # NRZ-20G mode,Power up TX PLL after prgramming PLL and before toggling FRAC_EN

            #### Enable Fractional PLLs after programming PLLs,# UPDATE 20180521
            wreg(c.tx_pll_frac_en_addr,     1, ln) # NRZ-20G mode, 0x0D7   [13] TX PLL FRAC_EN=1 after programming PLLs
            wreg(c.rx_pll_frac_en_addr,     1, ln) # NRZ-20G mode, 0x1F0   [13] RX PLL FRAC_EN=1 after programming PLLs   
            
            wreg(c.tx_mode10g_en_addr,      0, ln) # NRZ-20G mode, TX_NRZ_10G_EN=0 (or Disable NRZ Half-Rate Mode)
            wreg(c.rx_mode10g_addr,         0, ln) # NRZ-20G mode, RX_NRZ_10G_EN=0 (or Disable NRZ Half-Rate Mode)
            wreg(0x165,0x0000,ln)                  # NRZ-FullRate Mode, INTP Half Rate mode [0] = 0
            wreg(0x175,0xC6FF,ln)                  # NRZ-FullRate Mode, Margin Counter Phases 4321 en [7:4]=1111
            wreg(0x100,0x4900,ln)                  # NRZ-FullRate mode, F1F2_INIT_0, 0x5185
            wreg(0x107,0x44FB,ln)                  # NRZ-FullRate mode, F1F2_INIT_1, 0x5185
            wreg(0x183,0x4B08,ln)                  # NRZ-FullRate mode, F1F2_INIT_2, 0x5185
            wreg(0x184,0x82FA,ln)                  # NRZ-FullRate mode, F1F2_INIT_3, 0x9185
            wreg(0x185,0x4B82,ln)                  # NRZ-FullRate mode, F1F2_INIT_4, 0x5185
            wreg(0x186,0xCB88,ln)                  # NRZ-FullRate mode, F1F2_INIT_5, 0xD185
            wreg(0x187,0xC97A,ln)                  # NRZ-FullRate mode, F1F2_INIT_6, 0x5185
            wreg(0x188,0x417a,ln)                  # NRZ-FullRate mode, F1F2_INIT_7, 0x5185

        else: # NRZ 25.78125G, , NRZ Full-Rate Mode
            #### disable Fractional PLLs while programming PLLs,# UPDATE 20180521
            wreg(c.tx_pll_frac_en_addr,     0, ln) # NRZ-25G mode, 0x0D7   [13] TX PLL FRAC_EN=0 while programming PLLs
            wreg(c.rx_pll_frac_en_addr,     0, ln) # NRZ-25G mode, 0x1F0   [13] RX PLL FRAC_EN=0 while programming PLLs
        
            wreg(c.tx_pll_lvcocap_addr,    34, ln) # NRZ-25G mode, 0x0DB [14:8] TX PLL VCOCAP=33 or 34 for fvco=25.78125G                
            wreg(c.tx_pll_n_addr,          33, ln) # NRZ-25G mode, 0x0FE [15:7] TX_PLL N= 33*2*195.3125
            wreg(c.tx_pll_div4_addr,        0, ln) # NRZ-25G mode, 0x0FF    [0] TX_PLL DIV4=0
            wreg(c.tx_pll_div2_addr,        0, ln) # NRZ-25G mode, 0x0FF    [1] TX PLL DIV2=0
            wreg([0x0d9,[3,0]],             0, ln) # NRZ-25G mode, 0x0D9  [3:0] TX PLL FRAC_N[19:16]=0 (R2.0: 20 bits, R1.0: 16 bits)
            wreg(c.tx_pll_frac_n_addr,      0, ln) # NRZ-25G mode, 0x0D8 [15:0] TX PLL FRAC_N=0
            wreg(c.tx_pll_frac_order_addr,  0, ln) # NRZ-25G mode, 0x0D7[15:14] TX PLL FRAC_ORDER =0
                                                       
            wreg(c.rx_pll_lvcocap_addr,    34, ln) # NRZ-25G mode, 0x1F5 [15:9] RX PLL VCOCAP=33 or 34 for fvco=25.78125G                
            wreg(c.rx_pll_n_addr,          66, ln) # NRZ-25G mode, 0x1FD [15:7] RX_PLL N= 66*1*195.3125
            wreg(c.rx_pll_div4_addr,        0, ln) # NRZ-25G mode, 0x1FF    [6] RX_PLL DIV4=0
            wreg(c.rx_pll_div2_addr,        1, ln) # NRZ-25G mode, 0x1F4    [8] RX PLL DIV2=0
            wreg(c.rx_pll_frac_n_addr,      0, ln) # NRZ-25G mode, 0x1F1 [15:0] RX PLL FRAC_N=0
            wreg(c.rx_pll_frac_order_addr,  0, ln) # NRZ-25G mode, 0x1F0[15:14] RX PLL FRAC_ORDER =0
                                                       
            wreg(c.rx_pll_pu_addr,          1, ln) # NRZ-25G mode,Power up RX PLL after prgramming PLL and before toggling FRAC_EN
            wreg(c.tx_pll_pu_addr,          1, ln) # NRZ-25G mode,Power up TX PLL after prgramming PLL and before toggling FRAC_EN

            #### Enable Fractional PLLs after programming PLLs, if needed   # UPDATE 20180521
            wreg(c.tx_pll_frac_en_addr,     0, ln) # NRZ-25G mode, 0x0D7   [13] TX PLL FRAC_EN=0 kept off for 25G 
            wreg(c.rx_pll_frac_en_addr,     0, ln) # NRZ-25G mode, 0x1F0   [13] RX PLL FRAC_EN=0 kept off for 25G   

            wreg(c.tx_mode10g_en_addr,      0, ln) # NRZ-25G mode, TX_NRZ_10G_EN=0 (or Disable NRZ Half-Rate Mode)
            wreg(c.rx_mode10g_addr,         0, ln) # NRZ-25G mode, RX_NRZ_10G_EN=0 (or Disable NRZ Half-Rate Mode)
            wreg(0x165,0x0000,ln)                  # NRZ-FullRate Mode, INTP Half Rate mode [0] = 0
            wreg(0x175,0xC6FF,ln)                  # NRZ-FullRate Mode, Margin Counter Phases 4321 en [7:4]=1111
            wreg(0x100,0x4900,ln)                  # NRZ-FullRate mode, F1F2_INIT_0, 0x5185
            wreg(0x107,0x44FB,ln)                  # NRZ-FullRate mode, F1F2_INIT_1, 0x5185
            wreg(0x183,0x4B08,ln)                  # NRZ-FullRate mode, F1F2_INIT_2, 0x5185
            wreg(0x184,0x82FA,ln)                  # NRZ-FullRate mode, F1F2_INIT_3, 0x9185
            wreg(0x185,0x4B82,ln)                  # NRZ-FullRate mode, F1F2_INIT_4, 0x5185
            wreg(0x186,0xCB88,ln)                  # NRZ-FullRate mode, F1F2_INIT_5, 0xD185
            wreg(0x187,0xC97A,ln)                  # NRZ-FullRate mode, F1F2_INIT_6, 0x5185
            wreg(0x188,0x417a,ln)                  # NRZ-FullRate mode, F1F2_INIT_7, 0x5185
        '''
        wreg(0x165,0x0000,ln)                  # NRZ-FullRate Mode, INTP Half Rate mode [0] = 0
        wreg(0x175,0xC6FF,ln)                  # NRZ-FullRate Mode, Margin Counter Phases 4321 en [7:4]=1111
        wreg(0x100,0x4900,ln)                  # NRZ-FullRate mode, F1F2_INIT_0, 0x5185
        wreg(0x107,0x44FB,ln)                  # NRZ-FullRate mode, F1F2_INIT_1, 0x5185
        wreg(0x183,0x4B08,ln)                  # NRZ-FullRate mode, F1F2_INIT_2, 0x5185
        wreg(0x184,0x82FA,ln)                  # NRZ-FullRate mode, F1F2_INIT_3, 0x9185
        wreg(0x185,0x4B82,ln)                  # NRZ-FullRate mode, F1F2_INIT_4, 0x5185
        wreg(0x186,0xCB88,ln)                  # NRZ-FullRate mode, F1F2_INIT_5, 0xD185
        wreg(0x187,0xC97A,ln)                  # NRZ-FullRate mode, F1F2_INIT_6, 0x5185
        wreg(0x188,0x417a,ln)                  # NRZ-FullRate mode, F1F2_INIT_7, 0x5185
        '''
        
        if ln%2: # ODD lanes
            set_bandgap(bg_val=7,lane=ln)
        else:      # Even lanes
            set_bandgap(bg_val=2,lane=ln)
        
        ####################### NRZ mode, ln Reset                   
        if (input_mode=='dc'):
            wreg(c.rx_en_vcominbuf_addr,0,ln) # DC-Coupled mode 0x1E7[11]=0
        else:
            wreg(c.rx_en_vcominbuf_addr,1,ln) # AC-Coupled mode 0x1E7[11]=1
        
        # ---------------- LANE Q NRZ init end
        
        # Set the polarities of the lanes
        pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)

        #print("\ninit_lane_nrz(): Lane %s, Target DataRate=%3.5fG, Actual DataRate=%3.5f G"%(lane_name_list[ln],datarate,get_lane_pll(ln)[ln][0][0])),
        
    get_lane_mode(lanes) # update the Encoding modes of all lanes for this Slice        
    #for ln in lanes:
    #    print("\n init_lane_nrz(): Lane %s, Target DataRate=%3.5fG, Actual DataRate=%3.5f G"%(lane_name_list[ln],datarate,get_lane_pll(ln)[ln][0][0])),

    if not fw_loaded(print_en=0):
        for ln in lanes:
            lr(lane=ln)  

        
        #rx_monitor_clear(ln)
####################################################################################################
# 
# Initialize the target lane, or list of lanes, to PAM4 mode
####################################################################################################
def init_lane_pam4(datarate=None,input_mode='ac',lane=None):

    lanes = get_lane_list(lane)    
    global gEncodingMode
    c=Pam4Reg
    ### First put the target lanes' State Machine in reset mode while programming lane registers and PLLs
    for ln in lanes:
        wreg(c.rx_lane_rst_addr, 1, ln) # Keep lane in Reset while programming lane
        wreg(c.rx_pll_pu_addr,   0, ln) # Power down RX PLL while prgramming PLL
        wreg(c.tx_pll_pu_addr,   0, ln) # Power down RX PLL while prgramming PLL
    
    for ln in lanes:
        gEncodingMode[gSlice][ln] = ['pam4',53.125]
        lane_mode_list[ln] = 'pam4'
        ####disable AN/LT registers as default
        wreg(0x3000+(0x100*ln),0x0000,lane=0)
        wreg(0xe000+(0x100*ln),0xc000,lane=0)
        # ---------------- LANE Q PAM4 init begin
        ####################### Per-Lane PLLs ###
        wreg(0x1F0,0x0000,ln) # PAM4 mode, RX_PLL, Fractional PLL off  
        wreg(0x1F1,0x0000,ln) # PAM4 mode, RX_PLL, Fractional PLL off          
        wreg(0x0D7,0x0000,ln) # PAM4 mode, TX_PLL, Fractional PLL off  
        wreg(0x0D8,0x0000,ln) # PAM4 mode, TX_PLL, Fractional PLL off  

       #wreg(0x0FF,0x5df6,ln) # PAM4 mode, TX_BG bandgap | div4=0
       #wreg(0x0FE,0x223e,ln) # PAM4 mode, TX PLLN = 68*195.3125*4*2 | [3:1] //vcoi=0x7
        wreg(0x0FF,0x5df4,ln) # PAM4 mode, TX_BG bandgap | div4=0
        wreg(0x0FE,0x113e,ln) # PAM4 mode, TX PLLN = 34*195.3125*4 | [3:1] //vcoi=0x7
       #wreg(0x0FD,0x5636,ln) # PAM4 mode, TX PLL, bypass_pll=0
       #wreg(0x0FD,0x5436,ln) # PAM4 mode, TX PLL, bypass_pll=0, clear bypass 1p7 regulator 0xFD[9]
        wreg(0x0FC,0x7236,ln) # PAM4 mode, TX PLL, vvco_reg
        wreg(0x0FB,0x7fb6,ln) # PAM4 mode, TX PLL, vref_intp
        wreg(0x0FA,0x8010,ln) # PAM4 mode, TX PLL, testmode_tx_pll
        wreg(0x0DB,0x1e00,ln) # PAM4 mode, TX PLL, [14:8] // tx_vcocap=rx_vcocap=29 or 30 for fvco=26.5625G
        wreg(0x0DA,0x7de6,ln) # PAM4 mode, TX PLL, [14:10]// bm_vco=0x1f
        wreg(0x0D9,0xb760,ln)
                  
        wreg(0x1FF,0x5db1,ln) # PAM4 mode, RX_BG bandgap | [15:13]//vbg=0
       #wreg(0x1FC,0x1048,ln) # PAM4 mode, RX PLL, clear bypass 1p7 regulator 0x1FC[9]
       #wreg(0x1FD,0x213e,ln) # PAM4 mode, RX_PLLN= 33*2*195.3125 |  [3:1]  //vcoi=0x7
        wreg(0x1F5,0x3d40,ln) # PAM4 mode, RX PLL, lcvco_cap [15:9] //tx_vcocap=rx_vcocap=29 or 30 for fvco=26.5625G
        wreg(0x1FD,0x223e,ln) # PAM4 mode, RX_PLLN= 34*195.3125*4*2 [3:1]  //vcoi=0x7
       #wreg(0x1F5,0x3c40,ln) # PAM4 mode, RX PLL, lcvco_cap [15:9] //tx_vcocap=rx_vcocap=29 or 30 for fvco=26.5625G
       #wreg(0x1FD,0x113e,ln) # PAM4 mode, RX_PLLN= 34*195.3125*4*2 [3:1]  //vcoi=0x7
        wreg(0x1F4,0x7de6,ln) # PAM4 mode, RX PLL, en_div2=0 | [14:10]//bm_vco=0x1f
        wreg(0x1F3,0xb760,ln) # PAM4 mode, RX PLL, pu_intp_rx
        

        #### From Xiaofan 20171204
        wreg(0x0FD,0x1436,ln) # PAM4 mode, bypass_pll=1, bypass1p7reg=0
        wreg(0x0FB,0x6fb6,ln) # PAM4 mode, vref_intp, vref1p3vcodiv=3
        wreg(0x1FC,0x1448,ln) # PAM4 mode, bypass_pll=1, bypass1p7reg=0

        if ln%2: # ODD lanes
            set_bandgap(bg_val=7,lane=ln)
        else:      # Even lanes
            set_bandgap(bg_val=2,lane=ln)

        ####################### analog ###
        wreg(0x0f1,0x0007,ln) # PAM4 mode, pu_adc|pu_clkcomp|pu_clkreg
        wreg(0x0ef,0x9230,ln) # PAM4 mode, vrefadc
        wreg(0x0ee,0x691c,ln) # PAM4 mode, calnbs_top|bot
        wreg(0x0ed,0x8888,ln) # PAM4 mode, edge
        wreg(0x0eb,0x67fc,ln) # PAM4 mode, TX Swing
                  
        wreg(0x1f9,0x7e86,ln) # PAM4 mode, vrefagcdegen (was 0x7686)
        wreg(0x1f6,0x71b0,ln) # PAM4 mode, vagcbufdac
        wreg(0x1e7,0xc34c,ln) # PAM4 mode, pu_agc|pu_agcdl | en_vcominbuf=0
        
       
        wreg(0x1e6,0x88a5,ln) # PAM4 mode, bypass_agcreg
        wreg(0x1e5,0x6c38,ln) # PAM4 mode, 4e3c # PAM4 mode, vrefagc1p5reg
                  
        wreg(0x1da,0x6f00,ln) # PAM4 mode, 6d80 # clkphase0
                  
        wreg(0x1d4,0x01c0,ln) # PAM4 mode, agcgain1 | agacgain2
        wreg(0x1d5,0x0100,ln) # PAM4 mode, blwc_en
        wreg(0x1d6,0xcc00,ln) # PAM4 mode, en_az_short_pulse #rajan

        #######################  PAM4 Mode Configuration
        wreg(0x041,0x83df,ln) # PAM4 mode, up/dn mode OFF | pam4_en=1
        wreg(0x0b0,0x4000,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, (4000)
        wreg(0x0b0,0x4802,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, (4000)
        wreg(0x0b0,0x4000,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, (4000)
        wreg(0x1e7,0xc36c,ln) # PAM4 mode, ffe enabled in pam4 mode (agc_dl=1)

        #######################  PAM4 SM Configuration
        wreg(0x000,0x286b,ln)
        wreg(0x001,0xc000,ln) # changed from 0x8000 to 0xc000 per Yifei 20171203
        wreg(0x002,0x4000,ln)
        wreg(0x003,0x7873,ln)
        wreg(0x005,0xbd2a,ln)
        wreg(0x006,0x762b,ln)
        wreg(0x007,0x3ac2,ln)
        wreg(0x008,0xc001,ln)
        wreg(0x00a,0xe5b1,ln)
        wreg(0x00b,0x3d15,ln)
        wreg(0x00c,0x0080,ln)
        wreg(0x044,0x1035,ln)
        wreg(0x04b,0xe802,ln)
        wreg(0x079,0x00a4,ln)  #--- Turn On TED Qualifier 4/24/2018
        wreg(0x07b,0x4004,ln) #---- Set BLWC MU=4 4/24/2018
        wreg(0x087,0x0800,ln)
        
        # Updated rajan - need to verify
        wreg(0x005 ,0xbd29, ln)
        wreg(0x007 ,0x32bf, ln) 
        wreg(0x009 ,0x7665, ln)   
        
        #######################  PAM4 Rx optimize parameters
        sel_ctle_map(IL='ALL', lane=ln)
        ctle(7,ln)
       #wreg(0x048,0x2518,ln) # PAM4 mode, ctle_map
       #wreg(0x049,0x79eb,ln) # PAM4 mode, ctle_map
       #wreg(0x04a,0xbf3f,ln) # PAM4 mode, ctle_map, CTLE7=(7,7)
       #wreg(0x021,0x00f0,ln) # PAM4 mode, CTLE=7      
        wreg(0x020,0x03c0,ln) # PAM4 mode, timing loop|Kp=7 | Kf Set Kp=7 to compensate TED Qualifier ON 4/24/2018
        
        wreg(0x1d5,0x0100,ln) # PAM4 mode, bit8=1, see 0x007[14], baseline_ow en
        wreg(0x1e0,0xfc40,ln) # PAM4 mode, ffe_pol | pu_sum
        wreg(0x1e1,0x1000,ln) # PAM4 mode, ffe_gain12|ffe_sum4
        wreg(0x1e2,0x1601,ln) # PAM4 mode, ffe_k1 | ffe_sum3
        wreg(0x1e3,0x0101,ln) # PAM4 mode, ffe_k2 | ffe_sum2
        wreg(0x1e4,0x0101,ln) # PAM4 mode, ffe_s2| ffe_sum1      
        wreg(0x1df,0x6666,ln) # PAM4 mode, ffe1234_delay
        wreg(0x1de,0x77cc,ln) # PAM4 mode, ffe5678_delay
        
        ####################### Direct Connect (XSR)
        wreg(0x1dd,0xc1c2,ln) # PAM4 mode, ffe9_delay | en_skef = 0 | skef_value
        wreg(0x1d4,0x0260,ln) # PAM4 mode, 0d00 # PAM4 mode, agcgain1[15:9] (bin=4) / agcgain2 [8:4] (bin=31)
        wreg(0x004,0xb029,ln) # PAM4 mode, b029 f1_over_init
        wreg(0x012,0x2500,ln) # PAM4 mode, 3f80 Delta  
        wreg(0x0ed,0x7777,ln) # PAM4 mode, 7777 Edge
        
        ######################## Super Cal
        #wreg(0x004,0xb029,ln) # bit 0 =1
        wreg(0x077,0x4e5c,ln)
        wreg(0x078,0xe080,ln)
        wreg(0x009,0x8666,ln)
        wreg(0x087,0x0e00,ln) # super-cal enable | 0800 disable
        
        ####################### PAM4 mode, TX setting
        if gPam4TxSourceIsCredo:
            wreg(0x0a5,0x0200,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, pre2
            wreg(0x0a7,0xf800,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, pre1
            wreg(0x0a9,0x1100,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, main
            wreg(0x0ab,0x0000,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, post1
            wreg(0x0ad,0x0000,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, post2
            wreg(0x0af,0xfa08,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, tx_automode| MSB first|gc=en|Pre off|SHmode
            
        #######################  TX PRBS gen and Rx PRBS checker
        if gPrbsEn:
            wreg(0x0a0,0xe320,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, PRBS clock en before Patt en
            wreg(0x0a0,0xeb20,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, tx pam4 prbs31
            wreg(0x043,0x0cfa,ln) # PAM4 mode, Rx PAM4 prbs31 checker | MSB first
            wreg(0x042,0xb3fd,ln) # PAM4 mode, Rx PAM4 gc=en | Pre=dis
        else:
            wreg(0x0a0,0xe320,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, PRBS clock en before Patt en
            wreg(0x0a0,0x8320,lane=LanePartnerMap[gSlice][ln][1]) # PAM4 mode, TX pam4 FUNCTIONAL MODE
            wreg(0x043,0x0ce2,ln) # PAM4 mode, Rx PAM4 FUNCTIONAL MODE | MSB first
            wreg(0x042,0xb3fd,ln) # PAM4 mode, Rx PAM4 gc=en | Pre=dis
        
        ####################### PAM4 mode, ln Reset
        wreg(0x087,0x0800,ln) # super-cal disable        
        
        #if datarate!=None:
            #set_lane_pll(tgt_pll='both',datarate=datarate, div2=0, lane=ln)
        
        if chip_rev()==2.0:
            wreg([0x07b,[3]],  1, ln) # R2.0 PAM4 BLWC Pol = 1
            wreg([0x17c,[15]], 0, ln) # R2.0 NRZ BLWC en = 0 (disabled)
            
        # PAM4 50G: PLL = 53.125Gbps        
        #### disable Fractional PLLs while programming PLLs
        wreg(c.tx_pll_frac_en_addr,     0, ln) # PAM4-53G mode, 0x0D7   [13] TX PLL FRAC_EN=0
        wreg(c.rx_pll_frac_en_addr,     0, ln) # PAM4-53G mode, 0x1F0   [13] RX PLL FRAC_EN=0

        wreg(c.tx_pll_lvcocap_addr,    30, ln) # PAM4-53G mode, 0x0DB [14:8] TX PLL VCOCAP=30 for fvco=26.5625G                
        wreg(c.tx_pll_n_addr,          34, ln) # PAM4-53G mode, 0x0FE [15:7] TX_PLL N= 34*2*195.3125
        wreg(c.tx_pll_div4_addr,        0, ln) # PAM4-53G mode, 0x0FF    [0] TX_PLL DIV4=0
        wreg(c.tx_pll_div2_addr,        0, ln) # PAM4-53G mode, 0x0FF    [1] TX PLL DIV2=0
        wreg(c.tx_pll_frac_n_addr,      0, ln) # PAM4-53G mode, 0x0D8 [15:0] TX PLL FRAC_N=0
        wreg(c.tx_pll_frac_order_addr,  0, ln) # PAM4-53G mode, 0x0D7[15:14] TX PLL FRAC_ORDER =10

        wreg(c.rx_pll_lvcocap_addr,    30, ln) # PAM4-53G mode, 0x1F5 [15:9] RX PLL VCOCAP=30 for fvco=26.5625G                
        wreg(c.rx_pll_n_addr,          68, ln) # PAM4-53G mode, 0x1FD [15:7] RX_PLL N= 68*1*195.3125
        wreg(c.rx_pll_div4_addr,        0, ln) # PAM4-53G mode, 0x1FF    [6] RX_PLL DIV4=0
        wreg(c.rx_pll_div2_addr,        1, ln) # PAM4-53G mode, 0x1F4    [8] RX PLL DIV2=0
        wreg(c.rx_pll_frac_n_addr,      0, ln) # PAM4-53G mode, 0x1F1 [15:0] RX PLL FRAC_N=0
        wreg(c.rx_pll_frac_order_addr,  0, ln) # PAM4-53G mode, 0x1F0[15:14] RX PLL FRAC_ORDER =10
        
        wreg(c.rx_pll_pu_addr,          1, ln) # PAM4-53G mode,Power up RX PLL after prgramming PLL and before toggling FRAC_EN
        wreg(c.tx_pll_pu_addr,          1, ln) # PAM4-53G mode,Power up TX PLL after prgramming PLL and before toggling FRAC_EN

        #### Enable Fractional PLLs after programming PLLs, if needed
        wreg(c.tx_pll_frac_en_addr,     0, ln) # PAM4-53G mode, 0x0D7   [13] TX PLL FRAC_EN=0 kept off for 50G 
        wreg(c.rx_pll_frac_en_addr,     0, ln) # PAM4-53G mode, 0x1F0   [13] RX PLL FRAC_EN=0 kept off for 50G                

        if (input_mode=='dc'):
            wreg(c.rx_en_vcominbuf_addr,0,ln) # DC-Coupled mode 0x1E7[11]=0
        else:
            wreg(c.rx_en_vcominbuf_addr,1,ln) # AC-Coupled mode 0x1E7[11]=1
        
        # Set the polarities of the lanes
        pol(TxPolarityMap[gSlice][ln],RxPolarityMap[gSlice][ln],ln,0)
        # ---------------- LANE Q PAM4 init end
        
    get_lane_mode(lanes) # update the Encoding modes of all lanes for this Slice
    if not fw_loaded(print_en=0):
        for ln in lanes:
            lr(lane=ln)   

#################################################################################################### 
####################################################################################################
# Initialize lane to NRZ mode and then adapt RX
#
# 
#
####################################################################################################
def opt_lane_nrz(datarate=None, input_mode='ac',lane=None):
   
    global LanePartnerMap  # TX Source for RX, one  per lane per Slice
    global gChanEst; # Channel Estimates, one set per lane per Slice  
    lanes = get_lane_list(lane)
    #get_lane_mode(lanes) # update the Encoding modes of all lanes for this Slice
    c=NrzReg
    for ln in lanes:
        get_lane_mode(ln)
        
        if fw_loaded(print_en=0) == 1: 
            init_lane_for_fw(mode = 'nrz', rx_pol=RxPolarityMap[gSlice][ln],input_mode = input_mode,lane=ln)
            print 'init_lane_for_fw_nrz'
        else:init_lane_nrz(datarate, input_mode,ln) # NRZ mode
        line_encoding_mode = gEncodingMode[gSlice][ln][0]
        peer_encoding_mode = gEncodingMode[gSlice][LanePartnerMap[gSlice][ln][1]][0]
        ctle_map(0,1,3, lane=ln)
        ctle_map(1,1,5, lane=ln)
        ctle_map(2,1,7, lane=ln)
        ctle_map(3,2,7, lane=ln)
        ctle_map(4,3,7, lane=ln)
        ctle_map(5,4,7, lane=ln)
        ctle_map(6,5,7, lane=ln)
        ctle_map(7,7,7, lane=ln)
        
        if gNrzTxSourceIsCredo:  # Make sure TxPeer Lane is also the same mode and data rate as this lane (NRZ or PAM4)        
            if line_encoding_mode != peer_encoding_mode :
                print ("\nSlice %d Lane %s is in %s mode. Its TX Peer (%s) is in %s Mode"%(gSlice, lane_name_list[ln],line_encoding_mode.upper(),  lane_name_list[LanePartnerMap[gSlice][ln][1]],peer_encoding_mode.upper()))
                continue
            else:            
                tx_taps(0,-8,17,0,0,lane=LanePartnerMap[gSlice][ln][1])
                time.sleep(1)

        of, hf, chan_est = channel_analyzer_nrz(lane=ln)
        gChanEst[gSlice][ln]=[chan_est,of,hf]
        if chan_est == 0.0: # LANE FAILED Channel Estimation
            print ("\nSlice %d Lane %s (NRZ) Channel Analyzer: ChanEst: %6.3f <<< FAILED"%(gSlice, lane_name_list[ln],chan_est)),               
        else:
            print ("\nSlice %d Lane %s (NRZ) Channel Analyzer: ChanEst : %6.3f, OF: %2d, HF: %2d"%(gSlice,lane_name_list[ln],chan_est, of, hf)),       
                    
        if chan_est == 0.0:       # LANE FAILED
            agcgain(1,1,lane=ln) # initial DC Gain
        elif chan_est < 1.20:             # DIRECT LOOPBACK  ############## USE SR TABLE #################
            ctle(7, lane=ln)
            agcgain(1,28,lane=ln) # initial DC Gain
            #agcgain(1,4,lane=ln) # initial DC Gain
        elif chan_est < 1.40:
            ctle(6, lane=ln)
            agcgain(5,31,lane=ln)
        elif chan_est < 1.55:
            ctle(5, lane=ln)
            agcgain(10,31,lane=ln)
        elif chan_est < 1.80:
            ctle(4, lane=ln)
            agcgain(15,31,lane=ln)
        elif chan_est < 2.09:
            ctle(3, lane=ln)
            agcgain(25,31,lane=ln)
        elif chan_est < 2.70:
            ctle(2, lane=ln)
            agcgain(40,31,lane=ln)
        else:
            ctle(1, lane=ln)
            agcgain(60,31,lane=ln)
    
        tx_taps_table(chan_est,ln) # select TX Taps based on channel estimate
        if chan_est != 0: # Only if Lane passed Channel Estimation, finish the rest of adaptation
            lr(lane=ln) # reset lane
            #if (sig_det(lane=ln)[ln])==1: # if PHY RDY = 0, save time and skip the rest of adaptation
            ctle_search_nrz(lane=ln, t=0.02) 
            lr(lane=ln)
            cntr_tgt_nrz(tgt_val='low', lane=ln)
                
        serdes_params(lane=ln)         

####################################################################################################
def opt_lane_pam4(datarate=None,input_mode='ac',lane=None):

    global LanePartnerMap  # TX Source for RX, one  per lane per Slice
    global gChanEst; # Channel Estimates, one set per lane per Slice  
    lanes = get_lane_list(lane)
    #get_lane_mode(lanes) # update the Encoding modes of all lanes for this Slice
    c=Pam4Reg
    ########## DEVICE 0 EXTERNAL LOOPBACK LANES
    for ln in lanes:
        get_lane_mode(ln)
        init_lane_pam4(datarate,input_mode,ln) # PAM4 mode, BER ~1e7
        line_encoding_mode = gEncodingMode[gSlice][ln][0]
        peer_encoding_mode = gEncodingMode[gSlice][LanePartnerMap[gSlice][ln][1]][0]

   
        if gPam4TxSourceIsCredo:  # Make sure TxPeer Lane is also the same mode and data rate as this lane (NRZ or PAM4)    
            if line_encoding_mode != peer_encoding_mode :
                print ("\nSlice %d Lane %s is in %s mode. Its TX Peer (%s) is in %s Mode"%(gSlice, lane_name_list[ln],line_encoding_mode.upper(),  lane_name_list[LanePartnerMap[gSlice][ln][1]],peer_encoding_mode.upper()))
                continue
            else:  # Initialize This lane's TX Partner before getting Channel Estimate
                tx_taps(+5,-16,17,0,0,lane=LanePartnerMap[gSlice][ln][1])
                time.sleep(.1)
                
        dc_gain(22,31,8,1, ln)
        delta_ph(-1,ln)
        f13(3,ln)        
        lr(ln)
        opt_agc1,opt_agc2 = dc_gain_search(lane=ln, target_dac_val = 10)
        
        of,hf,chan_est= channel_analyzer_pam4(gain1_val = opt_agc1, gain2_val = opt_agc2, lane=ln)
        gChanEst[gSlice][ln]=[chan_est,of,hf]
        if chan_est == 0.0: # LANE FAILED Channel Estimation
            print ("\nSlice %d Lane %s (PAM4) Channel Analyzer: ChanEst FAILED <<<"%(gSlice, lane_name_list[ln])),
        else:
            print ("\nSlice %d Lane %s (PAM4) Channel Analyzer: ChanEst: %5.3f, OF: %2d, HF: %2d"%(gSlice, lane_name_list[ln],chan_est, of, hf)),             

        if chan_est < 1.80:  ############## USE SR TABLE #################
            ctle_map(0,7,1, lane=ln)
            ctle_map(1,3,4, lane=ln)
            ctle_map(2,3,5, lane=ln)
            ctle_map(3,6,3, lane=ln)
            ctle_map(4,4,6, lane=ln)
            ctle_map(5,7,5, lane=ln)
            ctle_map(6,6,7, lane=ln)
            ctle_map(7,7,7, lane=ln)
        
        else:               ############## USE LR TABLE #################
            ctle_map(0,2,1, lane=ln)  # unusual value  <--- just for testing
            ctle_map(1,3,1, lane=ln)
            ctle_map(2,2,2, lane=ln)
            ctle_map(3,4,1, lane=ln)
            ctle_map(4,5,1, lane=ln)
            ctle_map(5,6,1, lane=ln)
            ctle_map(6,7,1, lane=ln)
            ctle_map(7,3,4, lane=ln)
         
        if chan_est == 0.0:         # LANE FAILED Channel Estimation
            agcgain(1,1,lane=ln)    # initial DC Gain
        elif chan_est <1.25:        # DIRECT LOOPBACK  ############## USE SR TABLE #################
            ctle(7, lane=ln)
            delta_ph(4,ln)
            f13(1,ln)
            edge(4,4,4,4,ln)
            skef(1,3,ln)
            dc_gain(1,1,8,8, ln) # initial DC Gain
            ffe_taps(0x66,0x11,-0x11,0x11,0x01,0x01,ln) # initial FFE Taps 
        elif chan_est <1.36: # Artek 0%, SR, less than 10dB (7,5)
            ctle(7, lane=ln)
            delta_ph(2,ln)
            f13(3,ln)
            edge(4,4,4,4,ln)
            skef(1,4,ln)
            dc_gain(1,10,8,8, ln) # initial DC Gain
            ffe_taps(0x33,0x11,-0x11,0x12,0x01,0x01,ln)   
        elif chan_est <1.47: # Artek 0%, SR, less than 10dB (7,5)
            ctle(6, lane=ln)
            delta_ph(-6,ln)
            f13(4,ln)
            edge(6,6,6,6,ln)
            skef(1,5,ln)
            dc_gain(1,10,8,8, ln) # initial DC Gain
            ffe_taps(0x44,0x00,0x00,-0x11,0x01,0x01,ln)   
        elif chan_est <1.55:  # Artek 10%, MR - 1.57 (4,6)
            ctle(5, lane=ln)
            delta_ph(-6,ln)
            f13(4,ln)
            edge(6,6,6,6,ln)
            skef(1,5,ln)
            dc_gain(1,25,8,8, ln) # initial DC Gain
            ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,ln) # initial FFE Taps 
        elif chan_est <1.59:  # Artek 20%, MR,  ctle(4,6)
            ctle(4, lane=ln)
            delta_ph(-6,ln)
            f13(4,ln)
            edge(6,6,6,6,ln)
            skef(1,5,ln)
            dc_gain(8,31,8,8, ln) # initial DC Gain
            ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,ln) # initial FFE Taps
        elif chan_est <1.68:  # Artek 30%, MR, ctle(3,7)
            ctle(3, lane=ln)
            delta_ph(-6,ln)
            f13(5,ln)
            edge(9,9,9,9,ln)
            skef(1,6,ln)
            #dc_gain(10,31,8,8, ln) 
            ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,ln) # initial FFE Taps
        elif chan_est <1.80:  # Artek 40%,  ctle(3,5) ###### SR TABLE #############
            ctle(2, lane=ln)
            delta_ph(-8,ln)
            f13(5,ln)
            edge(10,10,10,10,ln)
            skef(1,6,ln)
            #dc_gain(15,31,8,8, ln)
            dc_gain(30,31,8,8, ln)             
            ffe_taps(0x77,0x11,0x11,-0x55,0x01,0x01,ln) # initial FFE Taps
        
        ################# NEW CTLE TABLE USED ####################### after this condition > 1.80
        elif chan_est <1.95:  # Artek 50% ctle(7,2)     
            ctle(6, lane=ln)
            delta_ph(-5,ln)
            f13(6,ln)
            edge(10,10,10,10,ln)
            skef(1,7,ln)
            #dc_gain(20,31,8,8, ln) 
            dc_gain(30,31,8,8, ln) 
            ffe_taps(0x77,0x11,0x11,-0x55,0x01,0x01,ln)
        elif chan_est <2.05:  # Artek 60% ctle(3,4)  #### NEW TABLE USED #####
            ctle(5, lane=ln)
            delta_ph(-5,ln)
            f13(6,ln)
            edge(11,11,11,11,ln)
            skef(1,7,ln)
            dc_gain(20,31,8,8, ln) 
            ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,ln)                       
        elif chan_est <2.25:  # Artek 70%, (6,2)
            ctle(4, lane=ln)
            delta_ph(-6,ln)
            f13(6,ln)
            edge(12,12,12,12,ln)
            skef(1,7,ln)
            #dc_gain(25,31,8,2, ln) 
            ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,ln) 
        elif chan_est <2.45:  # Artek 80% , (5,2)
            ctle(3, lane=ln)
            delta_ph(-7,ln)
            f13(7,ln)
            edge(12,12,12,12,ln)
            skef(1,7,ln)
            dc_gain(30,31,8,2, ln) 
            ffe_taps(0x77,0x01,0x11,-0x55,0x01,0x01,ln)                 
        elif chan_est <2.7:  # Artek 90%, (4,2)
            ctle(2, lane=ln)
            delta_ph(-8,ln)
            f13(8,ln)
            edge(13,13,13,13,ln)
            skef(1,7,ln)
            #dc_gain(35,31,8,2, ln)
            dc_gain(50,31,8,2, ln)            
            ffe_taps(0x77,0x01,0x11,0x11,0x01,0x01,ln) 
        elif chan_est <3.25: # Artek 100%, (6,1)
            ctle(1, lane=ln)
            delta_ph(-14,ln)
            f13(9,ln)
            edge(13,13,13,13,ln)
            skef(1,7,ln)
            #dc_gain(40,31,15,1, ln) 
            dc_gain(70,31,15,1, ln) 
            ffe_taps(0x11,0x11,0x11,0x11,0x32,0x30,ln)                   
        else: #if chan_est >=3.25: # Artek 100%, (6,1)
            #if gPam4TxSourceIsCredo: tx_taps(+3,-12,17,0,0,lane=LanePartnerMap[gSlice][ln][1])
            ctle(0, lane=ln)
            delta_ph(-14,ln)
            f13(9,ln)
            edge(13,13,13,13,ln)
            skef(1,7,ln)
            dc_gain(90,31,15,1, ln) 
            ffe_taps(0x11,0x55, 0x01,0x11,0x01,0x01,ln)
        
        print ("\nSlice %d Lane %s (PAM4) CTLE selection in EQ1: %d"%(gSlice, lane_name_list[ln],ctle2(lane=ln)))
        tx_taps_table(chan_est,ln) # select TX Taps based on channel estimate
        if chan_est != 0.0:         # LANE PASSED Channel Estimation
            #f13_table(lane=ln)
            #dc_gain_search(lane=ln,target_dac_val=10)
            lr(ln)
            
            #if sig_det(lane=ln)[ln]: # if PHY RDY = 0, save time and skip the rest of adaptation
            #lane_reset_fast(ln) # reset lanes
            time.sleep(1)
            
            wreg(c.rx_theta_update_mode_addr,0,ln) # Disable updn Mode
            delta_search(lane=ln, print_en=0) 
            #ctle_search(lane=ln, t=0.5)
            ctle_fine_search(lane=ln)
            if chan_est > 1.6:
                ctle_fine_search(lane=ln)
            #ctle_search(lane=ln, t=0.5)
            dc_gain_search(lane=ln)
            lr(ln)
            time.sleep(1)
            print ("\nSlice %d Lane %s (PAM4) CTLE selection in ctle_search: %d"%(gSlice, lane_name_list[ln],ctle2(lane=ln)))
            
            wreg(c.rx_theta_update_mode_addr,7,ln) # Enable updn Mode
            time.sleep(.1)
            delta_search(lane=ln, print_en=0) 
            f13_table(lane=ln)
            lr(ln)
            time.sleep(.5)            
            i=0
            list = [1,1,1,-1,-1,-1]
            while i < 5:
                if eye_check(lane=ln)[0] != 0:
                    i+=1
                    f13(val=(f13(lane=ln)[0]+list[i]),lane=ln)
                    lane_reset_fast(ln)
                    time.sleep(.3)
                else:
                    break
            print("\nSlice %d Lane %s final f13 value: %d"%(gSlice, lane_name_list[ln],f13(lane=ln)[0])), 
                
            if chan_est >=1.47: 
                ffe_search_a1_orig(lane=ln, print_en=0)
        
            lr(ln) # reset lanes
            time.sleep(.5)
            background_cal(enable='en', lane=ln)
            time.sleep(.5)
         
        serdes_params(lane=ln)   
          
####################################################################################################
def tx_taps_table(chan_est=None,lane=None):

    global LanePartnerMap  # TX Source for RX, one  per lane per Slice
    global gChanEst;  # Channel Estimates, one set per lane per Slice  
    lanes = get_lane_list(lane)
    #get_lane_mode(lanes) # update the Encoding modes of all lanes for this Slice
   
    for ln in lanes:
        get_lane_mode(ln)
        ##### Check if this lane is PAM4 or NRZ
        line_encoding_mode = gEncodingMode[gSlice][ln][0]
        peer_encoding_mode = gEncodingMode[gSlice][LanePartnerMap[gSlice][ln][1]][0]
        
        ##### Check if this lane's TX Peer is from Credo TX and both ends' mode (PAM4 or NRZ) match
        if (gPam4TxSourceIsCredo==1 or gNrzTxSourceIsCredo==1) and (line_encoding_mode!=peer_encoding_mode) :
            print ("\n***tx_taps_chan_est(): Slice %d Lane %s is in %s mode. Its TX Peer (%s) is in %s Mode"%(gSlice, lane_name_list[ln],line_encoding_mode.upper(),  lane_name_list[LanePartnerMap[gSlice][ln][1]],peer_encoding_mode.upper()))
            continue

        ##### If user does not pass Chan Estimate, get the chan estimate from most recent Rx Adaptation
        if chan_est==None:
            if fw_loaded(print_en=0):
                dbg_md = 2 if  line_encoding_mode=='pam4' else 1
                chanEst =(fw_debug_info(section=dbg_md, index=2,lane=ln)[ln]) / 256.0
                of      = fw_debug_info(section=dbg_md, index=4,lane=ln)[ln]
                hf      = fw_debug_info(section=dbg_md, index=5,lane=ln)[ln]            
                gChanEst[gSlice][ln]=[chanEst,of,hf]
                
            #print ("%5.3f,%2d,%2d" %(gChanEst[gSlice][ln][0],gChanEst[gSlice][ln][1],gChanEst[gSlice][ln][2])),
            chanEst = gChanEst[gSlice][ln][0]
        else:
            chanEst=chan_est
        
        ##### This lane is PAM4. Set the TX if the TX source for this lane is Credo TX
        if line_encoding_mode=='pam4' and gPam4TxSourceIsCredo==1:
            if   chanEst <0.90:  tx_taps(+2, -8,17,0,0,lane=LanePartnerMap[gSlice][ln][1]) # default or used for channel analyzer
            elif chanEst <1.25:  tx_taps(+1, -6,16,0,0,lane=LanePartnerMap[gSlice][ln][1]) # DIRECT LOOPBACK
            elif chanEst <1.36:  tx_taps(+1, -6,16,0,0,lane=LanePartnerMap[gSlice][ln][1]) # Artek 0%, SR, less than 10dB
            elif chanEst <1.47:  tx_taps(+2, -8,17,0,0,lane=LanePartnerMap[gSlice][ln][1]) # Artek 0%, SR, less than 10dB 
            elif chanEst <1.55:  tx_taps(+3,-12,17,0,0,lane=LanePartnerMap[gSlice][ln][1]) # Artek 10%, MR
            elif chanEst <1.59:  tx_taps(+3,-12,17,0,0,lane=LanePartnerMap[gSlice][ln][1]) # Artek 20%, MR
            elif chanEst <1.68:  tx_taps(+3,-12,17,0,0,lane=LanePartnerMap[gSlice][ln][1]) # Artek 30%, MR
            elif chanEst <1.80:  tx_taps(+3,-12,17,0,0,lane=LanePartnerMap[gSlice][ln][1]) # Artek 40%,
            elif chanEst <1.95:  tx_taps(+4,-12,17,0,0,lane=LanePartnerMap[gSlice][ln][1]) # Artek 50%    
            elif chanEst <2.05:  tx_taps(+4,-12,17,0,0,lane=LanePartnerMap[gSlice][ln][1]) # Artek 60%                   
            elif chanEst <2.25:  tx_taps(+4,-15,18,0,0,lane=LanePartnerMap[gSlice][ln][1]) # Artek 70%
            elif chanEst <2.45:  tx_taps(+4,-15,18,0,0,lane=LanePartnerMap[gSlice][ln][1]) # Artek 80%  LR
            elif chanEst <2.70:  tx_taps(+4,-15,18,0,0,lane=LanePartnerMap[gSlice][ln][1]) # Artek 90%  LR
            elif chanEst <3.20:  tx_taps(+4,-15,18,0,0,lane=LanePartnerMap[gSlice][ln][1]) # Artek 100% LR
            else:                tx_taps(+4,-15,18,0,0,lane=LanePartnerMap[gSlice][ln][1]) # 
                
        ###### This lane is NRZ. Set the TX if the TX source for this lane is Credo TX
        elif line_encoding_mode=='nrz' and gNrzTxSourceIsCredo==1:            
            if   chanEst < 0.90: tx_taps( 0, -8,17,0,0,lane=LanePartnerMap[gSlice][ln][1]) # default
            elif chanEst < 1.20: tx_taps( 0, -8,17,0,0,lane=LanePartnerMap[gSlice][ln][1])
            elif chanEst < 1.40: tx_taps( 0, -8,17,0,0,lane=LanePartnerMap[gSlice][ln][1])
            elif chanEst < 1.55: tx_taps( 0, -8,17,0,0,lane=LanePartnerMap[gSlice][ln][1])
            elif chanEst < 1.80: tx_taps( 0, -8,17,0,0,lane=LanePartnerMap[gSlice][ln][1])
            elif chanEst < 2.09: tx_taps( 0, -8,17,0,0,lane=LanePartnerMap[gSlice][ln][1])
            elif chanEst < 4.00: tx_taps( 0, -8,17,0,0,lane=LanePartnerMap[gSlice][ln][1])
            else:                tx_taps( 0, -8,17,0,0,lane=LanePartnerMap[gSlice][ln][1]) # default or used for channel analyzer

        tt=tx_taps(lane=LanePartnerMap[gSlice][ln])[LanePartnerMap[gSlice][ln][1]]
        diff = '-' if ln==LanePartnerMap[gSlice][ln][1] else '*'
        print ("\n  (%2d,%3d,%2d,%d,%d) %s-TX %s-->  %s-RX (ChanEst: %5.3f) "%(tt[0],tt[1],tt[2],tt[3],tt[4],lane_name_list[LanePartnerMap[gSlice][ln]], diff, lane_name_list[ln], chanEst)),   
####################################################################################################
def fw_serdes_params(lane=None,lc=0,ph=0,sl=None,print_en=1):

    fullstring = ""
    
    if not fw_loaded(print_en=0):
        print ("\n######## FW is Not Loaded ! #########"),
    
    lanes = get_lane_list(lane)
    
    if sl==None: # if slice is not defined used gSlice
        sl=gSlice 
    
    top_pll_A_cap = rreg([0x9501,[12,6]])
    top_pll_B_cap = rreg([0x9601,[12,6]])
    
    pam4_lane_in_this_slice=0
    for ln in lanes:
        if fw_loaded(print_en=0):
            [lane_mode,lane_speed]=fw_lane_speed(ln)[ln]
            gEncodingMode[gSlice][ln][0] = lane_mode
        else: # avoid crash if FW not loaded
            get_lane_mode(ln)
        if gEncodingMode[gSlice][ln][0] == 'pam4':
            pam4_lane_in_this_slice=1
            
    die_temp = temp_sensor_fast()        
            
    line_separator= "\n#+-----------------------------------------------------------------------------------------------------------------------------------------------------------------+"
    fullstring += line_separator
    fullstring +=  ("\n#| %5.1fC |    |    COUNTERS     |SD,Rdy,|FRQ |PLL Cal|  CHANNEL   |      CTLE      |  |   | EYE MARGIN  |         DFE       | TIMING |           FFE Taps         |"%die_temp)
    if pam4_lane_in_this_slice:
        fullstring += ("\n#|L,P,S,Ln|Mode| Adp ,ReAdp,LLost|AdpDone|PPM | %d,%d | Est ,OF,HF | Peaking, G1,G2 |SK|DAC|  1 , 2 , 3  | F0 , F1 ,F1/F0,F13|Del,Edge| K1 , K2 , K3 , K4 ,S1,S2,Sf|"%(top_pll_A_cap,top_pll_B_cap))
    else:
        fullstring += ("\n#|L,P,S,Ln|Mode| Adp ,ReAdp,LLost|AdpDone|PPM | %d,%d | Est ,OF,HF | Peaking, G1,G2 |SK|DAC|  1 , 2 , 3  |  F1,  F2,  F3     |Del,Edge| K1 , K2 , K3 , K4 ,S1,S2,Sf|"%(top_pll_A_cap,top_pll_B_cap))
    fullstring += line_separator
    
    for ln in lanes:
        if fw_loaded(print_en=0):
            [lane_mode,lane_speed]=fw_lane_speed(ln)[ln]
            gEncodingMode[gSlice][ln][0] = lane_mode
            gEncodingMode[gSlice][ln][1] = lane_speed
        else: # avoid crash if FW not loaded
            get_lane_mode(ln)
            lane_mode = gEncodingMode[gSlice][ln][0]
            lane_speed= int(gEncodingMode[gSlice][ln][1])
        
        if lane_mode.upper()=='OFF': # Lane is OFF
            fullstring += ("\n#|%d,%d,%d,%s| %3s|"%(lc,ph,sl, lane_name_list[ln], lane_speed))
            fullstring += ("                 |       |    |       |            |                |  |   |             |                   |        |                            |")

        else: # Lane is active, in PAM4 or NRZ mode            
            adapt_cnt = fw_adapt_cnt(ln)[ln]
            readapt_cnt = fw_readapt_cnt(ln)[ln]
            linklost_cnt = fw_link_lost_cnt(ln)[ln]
            sd = sig_det(ln)[ln]
            rdy = phy_rdy(ln)[ln]
            adapt_done = (rreg(c.fw_opt_done_addr) >> ln) & 1
            sd_flag  = '*' if ( sd!=1) else ' '
            rdy_flag = '*' if (rdy!=1 or adapt_done!=1) else ' '          
            ppm_val = ppm(ln)[ln]
            tx_cap = rreg(c.tx_pll_lvcocap_addr,ln)
            rx_cap = rreg(c.rx_pll_lvcocap_addr,ln)
            chan_est,of,hf = fw_chan_est(ln)[ln]
            chan_est,of,hf = gChanEst[gSlice][ln]
            ctle_val = ctle(lane=ln)[ln]
            ctle_1_bit4=rreg([0x1d7,[3]],ln)
            ctle_2_bit4=rreg([0x1d7,[2]],ln)
            ctle_1 = ctle_map(ctle_val, lane=ln)[ln][0] + (ctle_1_bit4*8)
            ctle_2 = ctle_map(ctle_val, lane=ln)[ln][1] + (ctle_2_bit4*8)
            #if (lane_mode.upper()=='PAM4' and chan_est < 1.80): ctle_val+=8
            if lane_mode.upper()=='PAM4' and (ctle_1_bit4==1 or ctle_2_bit4==1 or ctle_map(7)[ln][0]==7): ctle_val+=8
            skef_val = skef(lane=ln)[ln][1]
            dac_val  =  dac(lane=ln)[ln]

            delta_val = delta_ph(lane=ln)[0]
            edge1,edge2,edge3,edge4 = edge(lane=ln)[ln]
            eyes = eye(lane=ln)[ln]

            
            fullstring += ("\n#|%d,%d,%d,%s| %3s|%5d,%5d,%4d |%s%d,%d,%d%s|%3d | %2d,%2d "%(lc,ph,sl, lane_name_list[ln], lane_speed,adapt_cnt,readapt_cnt,linklost_cnt,sd_flag,sd,rdy,adapt_done,rdy_flag,ppm_val,tx_cap,rx_cap))  
            fullstring += ("|%5.2f,%2d,%2d " %(chan_est,of,hf))        
            fullstring += ("|%2d(%d,%-2d),%3d,%2d " %(ctle_val,ctle_1,ctle_2,dc_gain(lane=ln)[ln][0],dc_gain(lane=ln)[ln][1]))
            fullstring += ("|%d |%2d " %(skef_val,dac_val))
            
            if lane_mode.upper()=='PAM4': # Lane is in PAM4 mode
                f0,f1,f1f0_ratio = pam4_dfe(lane=ln)[ln]
                f13_val  = f13(lane=ln)[0]
                fullstring += ("|%4.0f,%3.0f,%3.0f "%(eyes[0],eyes[1],eyes[2]))               
                fullstring += ("|%4.2f,%4.2f,%5.2f,%2d " %(f0,f1,f1f0_ratio,f13_val)) 
                fullstring += ("|%3d,%X%X%X%X| " %(delta_val,edge1,edge2,edge3,edge4))
                [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin] = ffe_taps(lane=ln)[ln]
                fullstring +=('\b%4d,%4d,%4d,%4d,%02X,%02X,%02X| ' %(ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin))
            else: # Lane is in NRZ mode
                hf_0= fw_debug_cmd(section=1, index=250,lane=ln)
                hf_1= fw_debug_cmd(section=1, index=251,lane=ln)
                hf_2= fw_debug_cmd(section=1, index=252,lane=ln)
                     
                # of_0= fw_debug_cmd(section=1, index=255,lane=ln)
                # of_1= fw_debug_cmd(section=1, index=256,lane=ln)
                # of_2= fw_debug_cmd(section=1, index=257,lane=ln)
                
                # adc_cal_done_0= fw_debug_cmd(section=1, index=530,lane=ln)
                # adc_cal_done_1= fw_debug_cmd(section=1, index=531,lane=ln)
                # adc_cal_done_2= fw_debug_cmd(section=1, index=532,lane=ln)
                
                # read_B0_q_0= fw_debug_cmd(section=1, index=425, lane=ln)
                # read_B0_q_1= fw_debug_cmd(section=1, index=426, lane=ln)
                # read_B0_q_2= fw_debug_cmd(section=1, index=427, lane=ln)
                
                # agc_peaking_0= fw_debug_cmd(section=1, index=420, lane=ln)
                # agc_peaking_1= fw_debug_cmd(section=1, index=421, lane=ln)
                # agc_peaking_2= fw_debug_cmd(section=1, index=422, lane=ln)
                
                f1,f2,f3 = nrz_dfe(lane=ln)[ln]
                fullstring += ("|%4.0f         "%(eyes[0]))             
                fullstring += ("|%4d,%4d,%4d     " %(f1,f2,f3))
                fullstring += ("|%3d,%X%X%X%X| " %(delta_val,edge1,edge2,edge3,edge4)) 
                fullstring += ("                           | ")
                # print('|%2d %2d %2d %2d %2d %2d       |' %(hf_0, hf_1, hf_2, of_0, of_1, of_2) ),
                # if(hf_0<10 or hf_1<10 or hf_2<10 or of_0<10 or of_1<10 or of_2<10):
                    # print('\n<<<<< adc_cal_done = %2d %2d %2d <<<<<')%(adc_cal_done_0, adc_cal_done_1, adc_cal_done_2),
                    # print('\n<<<<< read_B0_q    = %2d %2d %2d <<<<<')%(read_B0_q_0, read_B0_q_1, read_B0_q_2),
                    # print('\n<<<<< agc peaking  = %2d %2d %2d <<<<<\n')%(agc_peaking_0, agc_peaking_1, agc_peaking_2),
                    # nrz_dump_fw(lane=ln)

            
        if (ln<lanes[-1] and ln==7) or (ln==lanes[-1]):
            fullstring += line_separator # Line separator between A lines and B lines
            
    if print_en == 1: 
        print fullstring
    else:
        return fullstring

####################################################################################################
def serdes_params(lane=None):

    top_pll_A_cap = rreg([0x9501,[12,6]])
    top_pll_B_cap = rreg([0x9601,[12,6]])

    line_separator= "\n#+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+"
    print line_separator,
    print ("\n#|   |    |Line | Data | RX |      TX Taps       |                 | SD, | FRQ |PLL Cal|  CHANNEL  |      CTLE     |   |   | EYE MARGIN  |         DFE       | TIMING  |           FFE Taps         |"),
    print ("\n#|Dev|Lane| Enc | Rate | In |Ln(-2, -1, M,+1,+2) | Pol,Gry,PrC,MSB | Rdy | PPM | %d,%d | Est,OF,HF |Peaking, G1,G2 |SK |DAC|  1 , 2 , 3  | F0 , F1 ,F1/F0,F13|Del,Edge | K1 , K2 , K3 , K4 ,S1,S2,Sf|"%(top_pll_A_cap,top_pll_B_cap)),
    print line_separator,

    lanes = get_lane_list(lane)
    get_lane_mode('all')
    
    for ln in lanes: 
        line_encoding = gEncodingMode[gSlice][ln][0].upper()
        #lane_speed= str(int(round((gEncodingMode[gSlice][ln][1]-1.0)/5.0)*5.0))+'G'
        lane_speed= gEncodingMode[gSlice][ln][1]
        tt=tx_taps(lane=LanePartnerMap[gSlice][ln][1])[LanePartnerMap[gSlice][ln][1]]
        tx_peer_marker = '<' if LanePartnerMap[gSlice][ln][1]==ln else '/'
        rx_mode='DC' if(rreg(c.rx_en_vcominbuf_addr,ln)==0) else 'ac'
        tx_pol, rx_pol = pol   (lane=ln,print_en=0)
        if line_encoding=='PAM4': # Lane is in PAM4 mode
            tx_gc , rx_gc  = gc    (lane=ln,print_en=0)
            tx_pc , rx_pc  = pc    (lane=ln,print_en=0)
            tx_msb, rx_msb = msblsb(lane=ln,print_en=0)
        sd=sig_det(ln)[ln]
        rdy=phy_rdy(ln)[ln]
        sd_flag ='*' if ( sd!=1) else ' '
        rdy_flag='*' if (rdy!=1) else ' '          
        ppm_val=ppm(ln)[ln]
        tx_cap= rreg(c.tx_pll_lvcocap_addr,ln)
        rx_cap= rreg(c.rx_pll_lvcocap_addr,ln)
        if fw_loaded(print_en=0): 
            fw_chan_est(ln)[ln]
        chan_est,of,hf = gChanEst[gSlice][ln]
        ctle_val = ctle(lane=ln)[ln]; ctle_1= ctle_map(ctle_val, lane=ln)[ln][0]; ctle_2= ctle_map(ctle_val, lane=ln)[ln][1]        
        skef_val = skef(lane=ln)[ln][1]
        dac_val  =  dac(lane=ln)[ln]
        delta_val= delta_ph(lane=ln)[0]
        edge1,edge2,edge3,edge4 = edge(lane=ln)[ln]
        eyes=eye(lane=ln)[ln]
        
        print ("\n#| %d | %s"%(gSlice, lane_name_list[ln])),  
        print ("|%4s |%6.3f| %s"%(line_encoding,lane_speed,rx_mode)),  
        print ("|%2s(%2d,%3d,%2d,%2d,%2d)"%(lane_name_list[LanePartnerMap[gSlice][ln][1]],tt[0],tt[1],tt[2],tt[3],tt[4])),   
       
        if line_encoding=='PAM4': # Lane is in PAM4 mode
            print ("| %d/%d,%d/%d,%d/%d,%d/%d" %(tx_pol, rx_pol,tx_gc , rx_gc,tx_pc , rx_pc,tx_msb, rx_msb)),
        else:
            print ("| %d/%d            " %(tx_pol, rx_pol)),
        print ("|%s%d,%d%s|%4d | %2d,%2d"%(sd_flag,sd,rdy,rdy_flag,ppm_val,tx_cap,rx_cap)),  
        print ("|%4.2f,%2d,%2d" %(abs(chan_est),of,hf)),        
        print ("| %d(%d,%d),%3d,%2d" %(ctle_val,ctle_1,ctle_2,dc_gain(lane=ln)[ln][0],dc_gain(lane=ln)[ln][1])),   
        print ("| %d |%2d" %(skef_val,dac_val)),   
        
        if line_encoding=='PAM4': # Lane is in PAM4 mode
            f0,f1,f1f0_ratio = pam4_dfe(lane=ln)[ln]
            f13_val  = f13(lane=ln)[0]
            print ("|%4.0f,%3.0f,%3.0f"%(eyes[0],eyes[1],eyes[2])),                
            print ("|%4.2f,%4.2f,%5.2f,%2d" %(abs(f0),abs(f1),abs(f1f0_ratio),f13_val)),   
            print ("|%3d,%X%X%X%X" %(delta_val,edge1,edge2,edge3,edge4)),   
            [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin] = ffe_taps(lane=ln)[ln]
            print('|%4d,%4d,%4d,%4d,%02X,%02X,%02X|' %(ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin)),
        else: # Lane is in NRZ mode
            print ("|%4.0f        "%(eyes[0])),                
            print ("|                   |        "),
            print('|                            |' ),
            
        if (ln<lanes[-1] and ln==7) or (ln==lanes[-1]):
            print line_separator, # Line separator between A lines and B lines
         
####################################################################################################
def fw_slice_params(lane=None,lc=0,ph=0,sl=None,print_en=1):

    if sl==None: # if slice number is not given use gSlice
        sl=gSlice
    else:
        sel_slice(sl)
        
    if not fw_loaded(print_en=0):
        print ("\n...Error in 'fw_slice_params': Slice %d has no FW!"%sl),
        return
        

    lanes = get_lane_list(lane)
           
              #[ 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 ,  0x08,  0x09,  0x0A, 0x0B]
    speed_list=['OFF','10G','20G','25G','26G','28G', '50G','07?', '50G', '50G', '56G', '0x0B?']
    mode_list =['OFF','NRZ-10G','NRZ-20G','NRZ-25G','NRZ-26G','NRZ-28G','PAM4-50G','pam4-07?','pam4-51G','PAM4-50G','pam4-56G','pam4-0B?']
    config_list={0x00: 'Not Configured', 
                 ## Retimer config codes
                 0x00: 'RetimerNrz - 0',        0x01: 'RetimerNrz - 1',        0x02: 'RetimerNrz - 2',       0x03: 'RetimerNrz - 3',
                 0x04: 'RetimerNrz - 4',        0x05: 'RetimerNrz - 5',        0x06: 'RetimerNrz - 6',       0x07: 'RetimerNrz - 7',
                 0x10: 'RetimerPam4 - 0',       0x11: 'RetimerPam4 - 1',       0x12: 'RetimerPam4 - 2',      0x13: 'RetimerPam4 - 3',
                 0x14: 'RetimerPam4 - 4',       0x15: 'RetimerPam4 - 5',       0x16: 'RetimerPam4 - 6',      0x17: 'RetimerPam4 - 7',
                 ## Retimer Cross-Mode config codes
                 0x20: 'RetimerNrz X 0',        0x21: 'RetimerNrz X 1',        0x22: 'RetimerNrz X 2',       0x23: 'RetimerNrz X 3',                    
                 0x24: 'RetimerNrz X 4',        0x25: 'RetimerNrz X 5',        0x26: 'RetimerNrz X 6',       0x27: 'RetimerNrz X 7',
                 0x30: 'RetimerPam4 X 0',       0x31: 'RetimerPam4 X 1',       0x32: 'RetimerPam4 X 2',      0x33: 'RetimerPam4 X 3',
                 0x34: 'RetimerPam4 X 4',       0x35: 'RetimerPam4 X 5',       0x36: 'RetimerPam4 X 6',      0x37: 'RetimerPam4 X 7',
                 ## Bitmux config codes
                 0x40: 'BitMux-20G - 0',        0x41: 'BitMux-20G - 1',        0x42: 'BitMux-20G - 2',       0x43: 'BitMux-20G - 3',        0x44: 'BitMux-20G - 4',       0x45: 'BitMux-20G - 5', 
                 0x50: 'BitMux-53G - 0',        0x51: 'BitMux-53G - 1',        0x52: 'BitMux-53G - 2',       0x53: 'BitMux-53G - 3',        0x54: 'BitMux-53G - 4',       0x55: 'BitMux-53G - 5', 
                 ## Retimer Cross-Mode config codes
                 0x90: 'GearBox100G - 0',       0x91: 'GearBox100G - 1',       0x92: 'GearBox100G - 2', 
                 0x98: 'GearBox100gNoFecB - 0', 0x99: 'GearBox100gNoFecB - 1', 0x9a: 'GearBox100gNoFecB - 2',0x9a: 'GearBox100gNoFecB - 3',
                 0xb0: 'GearBox50G - 0',        0xb1: 'GearBox50G - 1',        0xb2: 'GearBox50G - 2',       0xb3: 'GearBox50G - 3',
                 0xb8: 'GearBox50gNoFecB - 0',  0xb9: 'GearBox50gNoFecB - 1',  0xba: 'GearBox50gNoFecB - 2', 0xba: 'GearBox50gNoFecB - 3',
                 ## Serdes-Only config codes
                 0xC0: 'PhyOnlyNRZ', 0xD0: 'PhyOnlyPAM4', 
                }
    lane_status_list=['NotRdy*','RDY']
    lane_optic_mode_list=['Off','ON']

    result = {} 

    line_separator=    "\n +------------------------------------------------------------------------+"
    if print_en: print("\n              FW Configuration For Slice: (%d,%d,%d)"%(lc,ph,sl)),
    #if print_en: print lanes,
    if print_en: print line_separator,
    if print_en: print("\n | LC,Ph,Sl, Ln |     Config - Group    | Mode-Speed | OpticBit |  Status |"),
    if print_en: print line_separator,
    
    for ln in lanes:    
        speed_index     = fw_debug_cmd(section=0,index=4,lane=ln)
        #if speed_index > len(speed_list)-1: speed_index=0
        lane_speed      = speed_list[speed_index]
        lane_mode_speed = mode_list[speed_index]
        config_code_this_lane= fw_debug_cmd(section=0, index=38, lane=ln)
        lane_config     = config_list[config_code_this_lane] if config_code_this_lane<0xC0 else config_list[config_code_this_lane & 0xF0]
        lane_optic_mode = lane_optic_mode_list[(fw_reg_rd(20) >> ln) & 1]
        
        if 'OFF' in lane_mode_speed: lane_config = 'None' # overwrite config to 'None' is lane is 'OFF'
        
        #### GB or BM's status for all lanes is obtained through fw debug command section 0 index 40
        if 'GearBox' in lane_config or 'BitMux' in lane_config:
            lane_rdy   = lane_status_list[(fw_debug_cmd(section=0, index=40, lane=ln) >> ln) & 1]
        #### PHY Mode's status for all lanes is obtained through register 0x98c9
        else: 
            lane_rdy   =  lane_status_list[(rreg(c.fw_opt_done_addr) >> ln) & 1]
            

        result[ln]  = [lane_config,lane_mode_speed,lane_optic_mode,lane_rdy]
        
        if print_en: print("\n | %2d,%2d,%2d, %2s | %s |  %s  | %s | %s |"%(lc,ph,sl,lane_name_list[ln],lane_config.center(21,' '),lane_mode_speed.center(8,' '),lane_optic_mode.center(8,' '),lane_rdy.center(7,' '))),
    
    if print_en: print line_separator

    if not print_en: return result
####################################################################################################
# 
# Set Lanes in PRBS or Functional Mode
# prbs_mode options: 'functional'  : disable prbs, used in loopback, retimer or gearbox modes with no FEC
#                    'prbs'        : enable prbs generator, leaves the prbs pattern unchanged
#                    'prbs31       : enable prbs generator, select prbs pattern PRBS31
#                    'prbs23       : enable prbs generator, select prbs pattern PRBS23 (NRZ only)
#                    'prbs15       : enable prbs generator, select prbs pattern PRBS15
#                    'prbs13       : enable prbs generator, select prbs pattern PRBS13 (PAM4 only)
#                    'prbs9        : enable prbs generator, select prbs pattern PRBS9
####################################################################################################
def prbs_mode_select(lane=None, prbs_mode='prbs'):
    
    lanes = get_lane_list(lane)
    if prbs_mode.upper() == 'PRBS': prbs_mode ='PRBS31'
    
    #global gPrbsEn;  gPrbsEn = prbs_mode
   
    nrz_prbs_pat  = ['PRBS9', 'PRBS15', 'PRBS23', 'PRBS31'] 
    pam4_prbs_pat = ['PRBS9', 'PRBS13', 'PRBS15', 'PRBS31']
    
    # if a specific prbs pattern is requested, program the PRBS type register bits
    if 'PRBS' in prbs_mode.upper() :
        for ln in lanes:
            wreg([0x0a0,  [9,8]],pam4_prbs_pat.index(prbs_mode.upper()),ln) # PAM4/NRZ TX PRBS Pattern
            if gEncodingMode[gSlice][ln][0]=='pam4':
                wreg([0x043,  [6,5]],pam4_prbs_pat.index(prbs_mode.upper()),ln) # PAM4 RX PRBS Pattern
            else:
                wreg([0x161,[13,12]], nrz_prbs_pat.index(prbs_mode.upper()),ln) # NRZ RX PRBS Pattern
    
    for ln in lanes:
        get_lane_mode(ln)
        ###### Lane is in PAM4 mode ####################
        if gEncodingMode[gSlice][ln][0]=='pam4':
            wreg([0x0b0,[14]],0,ln)    # PAM4 mode, NRZ PRBS Gen clock dis
            wreg([0x0b0,[11]],0,ln)    # PAM4 mode, NRZ PRBS dis
            wreg([0x0b0, [1]],0,ln)    # PAM4 mode, NRZ TX dis
            wreg([0x0b0, [0]],0,ln)    # PAM4 mode, NRZ 10G dis
            
           #wreg([0x0a0, [6]],0,ln)    # PAM4 mode, TX pol flip = 0, leave polarity as it was
           #wreg([0x0a0, [5]],0,ln)    # PAM4 mode, TX ana pol flip = 0, leave polarity as it was
            
            ######## Credo PAM4 TX in Functional Mode, used in loopback, retimer and gearbox modes
            if 'PRBS' not in prbs_mode.upper(): 
                wreg([0x0a0,[15]],0,ln)# PAM4 mode, PRBS Gen de-selected
                wreg([0x0a0,[14]],0,ln)# PAM4 mode, PRBS Gen clock dis
                wreg([0x0a0,[13]],0,ln)# PAM4 mode, PRBS Gen test data dis
                wreg([0x0a0,[11]],0,ln)# PAM4 mode, PRBS Gen dis
                
               #wreg([0x043,  [7]],1,ln)# PAM4 mode, PRBS Checker flip = 1, leave polarity as it was
               #wreg([0x043,[6,5]],3,ln)# PAM4 mode, PRBS Checker PRBS pattern, done already, see above
                wreg([0x043,  [4]],1,ln)# PAM4 mode, PRBS Checker powered up
                wreg([0x043,  [3]],1,ln)# PAM4 mode, PRBS Sync Checker powered up
                wreg([0x043,  [1]],1,ln)# PAM4 mode, PRBS Sync Checker auto-sync en  
                #wreg(0x043,0x0ce2,ln) # PAM4 mode, Rx PAM4 FUNCTIONAL MODE | MSB first
            
            ######## Credo PAM4 TX and RX in PRBS Mode
            else: 
                wreg([0x0a0,[15]],1,ln) # PAM4 mode, PRBS Gen selected
                wreg([0x0a0,[14]],1,ln) # PAM4 mode, PRBS Gen clock en
                wreg([0x0a0,[13]],1,ln) # PAM4 mode, PRBS Gen test data en
                wreg([0x0a0,[11]],1,ln) # PAM4 mode, PRBS Gen en
                
               #wreg([0x043,  [7]],1,ln)# PAM4 mode, PRBS Checker flip = 1, leave polarity as it was
               #wreg([0x043,[6,5]],3,ln)# PAM4 mode, PRBS Checker PRBS pattern, done already, see above
                wreg([0x043,  [4]],1,ln)# PAM4 mode, PRBS Checker powered up
                wreg([0x043,  [3]],1,ln)# PAM4 mode, PRBS Sync Checker powered up
                wreg([0x043,  [1]],1,ln)# PAM4 mode, PRBS Sync Checker auto-sync en  
                
                #wreg(0x043,0x0cfa,ln)  # PAM4 mode, Rx PAM4 prbs31 checker | MSB first

        ###### Lane is in NRZ mode #######################
        else: 
            wreg([0x0b0,[14]],1,ln)    # NRZ mode, NRZ PRBS Gen clock en
            wreg([0x0b0,[11]],1,ln)    # NRZ mode, NRZ PRBS en
            wreg([0x0b0, [1]],1,ln)    # NRZ mode, NRZ TX en
           #wreg([0x0b0, [0]],0,ln)    # NRZ mode, NRZ TX 10G leave as it was
           
           #wreg([0x0a0, [6]],1,ln)    # NRZ mode, TX pol flip1 = leave as it was
           #wreg([0x0a0, [5]],0,ln)    # NRZ mode, TX pol flip2 = leave as it was
            
            ######## Credo NRZ TX in Functional Mode, used in loopback, retimer and gearbox modes
            if 'PRBS' not in prbs_mode.upper(): 
                wreg([0x0a0,   [15]],0,ln)# NRZ mode, PRBS Gen de-selected
                wreg([0x0a0,   [14]],0,ln)# NRZ mode, PRBS Gen clock dis
                wreg([0x0a0,   [13]],0,ln)# NRZ mode, PRBS Gen test data dis
                wreg([0x0a0,   [11]],0,ln)# NRZ mode, PRBS Gen dis
               #wreg([0x161,   [14]],1,ln)# NRZ mode, PRBS Checker flip = 1, leave as it was
               #wreg([0x161,[13,12]],3,ln)# NRZ mode, PRBS Checker PRBS pattern, done already, see above
               #wreg([0x161,   [10]],0,ln)# NRZ mode, PRBS Checker powered off

                #wreg(0x0a0,0x0120,ln) # NRZ mode, TX functional mode
                #wreg(0x161,0x7120,ln) # NRZ mode, PRBS Rx checker off, bit 10
                
            ######## Credo NRZ TX and RX in PRBS Mode
            else: 
                wreg([0x0a0,   [15]],1,ln)# NRZ mode, PRBS Gen selected
                wreg([0x0a0,   [14]],1,ln)# NRZ mode, PRBS Gen clock en
                wreg([0x0a0,   [13]],1,ln)# NRZ mode, PRBS Gen test data en
                wreg([0x0a0,   [11]],1,ln)# NRZ mode, PRBS Gen en
               #wreg([0x161,   [14]],1,ln)# NRZ mode, PRBS Checker flip = 1, leave as it was
               #wreg([0x161,[13,12]],3,ln)# NRZ mode, PRBS Checker PRBS pattern, done already, see above
                wreg([0x161,   [10]],1,ln)# NRZ mode, PRBS Checker powered up
                #wreg(0x161,0x7520,ln)    # NRZ mode, PRBS checker up, bit 10

####################################################################################################
def rx_monitor_clear(lane=None, fec_thresh=15):
    global gLaneStats #[per Slice][per lane], [PrbsCount, PrbsCount-1,PrbsCount-2, PrbsRstTime, PrbsLastReadoutTime]
    global gSlice
    gSlice=sel_slice()
    lanes = get_lane_list(lane)
    
    for ln in lanes:
        get_lane_mode(ln)
        c = Pam4Reg if gEncodingMode[gSlice][ln][0].lower() == 'pam4' else NrzReg

        ###### 1. Initialize FEC Analyzer for this lane
        if gEncodingMode[gSlice][ln][0].lower() == 'pam4':
            fec_ana_init(lane=ln, delay=.1, err_type=0, T=fec_thresh, M=10, N=5440, print_en=0)
        else: # Lane is in NRZ mode 
            fec_ana_init(lane=ln, delay=.1, err_type=0, T= 7, M=10, N=5280, print_en=0)

        ###### 2. Clear Rx PRBS Counter for this lane
        wreg(c.rx_err_cntr_rst_addr, 0, ln)
        time.sleep(0.001)
        wreg(c.rx_err_cntr_rst_addr, 1, ln)
        time.sleep(0.001)
        wreg(c.rx_err_cntr_rst_addr, 0, ln)
        
        ###### 3. Capture Time Stamp for Clearing FEC and PRBS Counters. Used for Calculating BER
        prbs_reset_time = time.time() # get the time-stamp for the counter clearing

        ###### 4. Clear Stats for this lane
        #                     prbs1/2/3                               eye123     fec1,2
        gLaneStats[gSlice][ln]=[long(0),long(0),long(0),prbs_reset_time, prbs_reset_time,0,0,0,'CLR',0,0]
                               #[0,0,0,prbs_reset_time, prbs_reset_time,0,0,0,'RDY',0,0,0,0]
    return gLaneStats[gSlice]
    
########################################### 
def rx_monitor_capture (lane=None): 
    global gLaneStats #[per Slice][per lane], [PrbsCount, PrbsCount-1,PrbsCount-2, PrbsRstTime, PrbsLastReadoutTime]
    global gSlice
    gSlice=sel_slice()
    lanes = get_lane_list(lane)
   
    for ln in lanes:
        get_lane_mode(ln)
        c = Pam4Reg if gEncodingMode[gSlice][ln][0].lower() == 'pam4' else NrzReg
        
        ###### 1. Capture FEC Analyzer Data for this lane
        tei = fec_ana_tei(lane=ln)
        teo = fec_ana_teo(lane=ln)
        #sei = fec_ana_sei(lane=ln)
        #bei = fec_ana_bei(lane=ln)

        ###### 2. Capture PRBS Counter for this lane
        cnt1 = long(rreg(c.rx_err_cntr_msb_addr, ln)<<16) + rreg(c.rx_err_cntr_lsb_addr, ln)
        cnt = long(rreg(c.rx_err_cntr_msb_addr, ln)<<16) + rreg(c.rx_err_cntr_lsb_addr, ln)
        if cnt<cnt1: 
            cnt=cnt1 # check for LBS 16-bit wrap around
        
        ###### 3. Capture Time Stamp for the FEC and PRBS Counters. Used for Calculating BER
        cnt_time = time.time() # PrbsReadoutTime = get the time stamp ASAP for valid prbs count
        
        sd = sig_det(ln)[ln]
        rdy = phy_rdy(ln)[ln]
        adapt_done = (rreg(c.fw_opt_done_addr) >> ln) & 1
        cnt_n1 = long(gLaneStats[gSlice][ln][0]) # PrevPrbsCount-1
        cnt_n2 = long(gLaneStats[gSlice][ln][1]) # PrevPrbsCount-2
        if gLaneStats[gSlice][ln][3] != 0:  #if the PRBS count was cleared at least once before, use the time of last clear
            prbs_reset_time = gLaneStats[gSlice][ln][3]
        else:
            prbs_reset_time = cnt_time #if the PRBS counter was never cleared before, use the current time as the new 'clear time'
        link_status = 'RDY'
        if(sd==0 or rdy ==0 or adapt_done==0): # check for PHY_RDY first
            tei=0xEEEEEEEEL 
            teo=0xEEEEEEEEL 
            cnt=0xEEEEEEEEL # return an artificially large number if RDY=0
            cnt_n1 = long(0) # PrevPrbsCount-1
            cnt_n2 = long(0) # PrevPrbsCount-2
            link_status = 'NOT_RDY'
            eyes=[-1,-1,-1]
        else: # RDY=1, then consider the PRBS count value
            eyes=eye(lane=ln)[ln]          
            # if(cnt < cnt_n1): # check for PRBS counter wrap-around
                # cnt = 0xFFFFFFFEL # return an artificially large number if counter rolls over
                # cnt_n1 = long(0) # PrevPrbsCount-1
                # cnt_n2 = long(0) # PrevPrbsCount-2

                
        # if RDY=1 and PRBS count value is considered a valid count
        gLaneStats[gSlice][ln]=[cnt,cnt_n1,cnt_n2,prbs_reset_time, cnt_time,eyes[0],eyes[1],eyes[2],link_status,tei,teo]

    return gLaneStats[gSlice]

########################################### 
def rx_monitor_print (lane=None,lc=0,ph=0,sl=None): 
    global gLaneStats #[per Slice][per lane], [PrbsCount, PrbsCount-1,PrbsCount-2, PrbsRstTime, PrbsLastReadoutTime]
    myber=99e-1
    global gSlice
    gSlice=sel_slice()
    
    lanes = get_lane_list(lane)

    Slice=gSlice
    get_lane_mode(lanes)
    if sl==None: # if slice is not defined used gSlice
        sl=gSlice
    
    skip="       -"
    line="\n------------- "
    for ln in lanes:
        if ln == 8 :
            line+="|------- "
        else:
            line+="-------- "
    print line,
    
#    print("\n       Slice"),
    #print("\n LC,Phy,Slice"),
    #for ln in lanes:  print("   %d,%d,%d" %(lc,ph,sl)),
    print("\n   Slice_Lane"),
    for ln in lanes:  print("   S%d_%s" %(sl,lane_name_list[ln])),
        
    print("\n     Encoding"),
    for ln in lanes:  print("%8s" %(gEncodingMode[gSlice][ln][0].upper())),
    print("\nDataRate Gbps"),
    for ln in lanes:  print("%8.4f" %(gEncodingMode[gSlice][ln][1])),

    print line,
    print("\n  Link Status"),
    for ln in lanes:  print("%8s" %(gLaneStats[gSlice][ln][8])),
    print("\n    Eye1 (mV)"),
    for ln in lanes:  
        if(gLaneStats[gSlice][ln][8] == 'RDY'):
            print ("%8.0f"%(gLaneStats[gSlice][ln][5])),                
        else:
            print skip,
    print("\n    Eye2 (mV)"),
    for ln in lanes:  
        if(gEncodingMode[gSlice][ln][0].upper()== 'PAM4' and gLaneStats[gSlice][ln][8] == 'RDY'):
            print ("%8.0f"%(gLaneStats[gSlice][ln][6])),                
        else:
            print skip,
    print("\n    Eye3 (mV)"),
    for ln in lanes:  
        if(gEncodingMode[gSlice][ln][0].upper()== 'PAM4' and gLaneStats[gSlice][ln][8] == 'RDY'):
            print ("%8.0f"%(gLaneStats[gSlice][ln][7])),                
        else:
            print skip,
        
    print line,
    print("\n Elapsed Time"),
    for ln in lanes:
        if(gLaneStats[gSlice][ln][8] == 'RDY'):
            elapsed_time=gLaneStats[gSlice][ln][4] - gLaneStats[gSlice][ln][3]    
            if elapsed_time < 1000.0:
                print("%6.1f s" % (elapsed_time) ),
            else:
                print("%6.0f s" % (elapsed_time) ),
        else:
            print skip,
    
    # print("\nPrev PRBS    "),
    # for ln in lanes:  print("%8X " % (gLaneStats[gSlice][ln][1]) ),
    #print("\nCurr PRBS    "),
    print("\n     PRBS Cnt"),
    for ln in lanes:
        if(gLaneStats[gSlice][ln][8] == 'RDY'):
            print("%8X" % (gLaneStats[gSlice][ln][0]) ),
        else:
            print skip,
    # print("\nDeltaPRBS    "),
    # for ln in lanes:  print("%8X " % (gLaneStats[gSlice][ln][0] - gLaneStats[gSlice][ln][1]) ),

    print("\n     PRBS BER"),
    for ln in lanes:
        curr_prbs_cnt = float(gLaneStats[gSlice][ln][0])
        prbs_accum_time = gLaneStats[gSlice][ln][4] - gLaneStats[gSlice][ln][3]
        data_rate = gEncodingMode[gSlice][ln][1]
        bits_transferred = float((prbs_accum_time*data_rate*pow(10,9)))       
        if curr_prbs_cnt==0 or bits_transferred==0: 
            ber_val = 0
            print("%8d" % (ber_val) ),
            myber= ((ber_val) )
        else:
            ber_val= curr_prbs_cnt / bits_transferred
            if(gLaneStats[gSlice][ln][8] == 'RDY'):
                 print("%8.1e" % (ber_val) ),
                 myber= ((ber_val) )
            else:
                print skip,

            
    print line,
    #print "\n  User has set the FEC Threshold to less than max"
    if (gEncodingMode[gSlice][ln][0].lower() == 'pam4' and gFecThresh<15) or\
       (gEncodingMode[gSlice][ln][0].lower() != 'pam4' and gFecThresh<7):
        print("\n   FEC Thresh"),
        for ln in lanes:
            print("%8d" % (gFecThresh) ),

    print("\n  pre-FEC Cnt"),
    for ln in lanes:  
        if(gLaneStats[gSlice][ln][8] == 'RDY'):
            print("%8X" % (gLaneStats[gSlice][ln][9]) ),
        else:
            print skip,

    print("\n Post-FEC Cnt"),
    for ln in lanes:  
        if(gLaneStats[gSlice][ln][8] == 'RDY'):
            print("%8X" % (gLaneStats[gSlice][ln][10]) ),
        else:
            print skip,
    print("\n  pre-FEC BER"),
    for ln in lanes:
        curr_prbs_cnt = float(gLaneStats[gSlice][ln][9])
        prbs_accum_time = gLaneStats[gSlice][ln][4] - gLaneStats[gSlice][ln][3]
        data_rate = gEncodingMode[gSlice][ln][1]
        bits_transferred = float((prbs_accum_time*data_rate*pow(10,9)))       
        if curr_prbs_cnt==0 or bits_transferred==0: 
            ber_val = 0
            print("%8d" % (ber_val) ),
        else:
            ber_val= curr_prbs_cnt / bits_transferred
            if(gLaneStats[gSlice][ln][8] == 'RDY'):
                print("%8.1e" % (ber_val) ),
            else:

                print skip,
    print("\n Post-FEC BER"),
    for ln in lanes:
        curr_prbs_cnt = float(gLaneStats[gSlice][ln][10])
        prbs_accum_time = gLaneStats[gSlice][ln][4] - gLaneStats[gSlice][ln][3]
        data_rate = gEncodingMode[gSlice][ln][1]
        bits_transferred = float((prbs_accum_time*data_rate*pow(10,9)))       
        if curr_prbs_cnt==0 or bits_transferred==0: 
            ber_val = 0
            print("%8d" % (ber_val) ),
        else:
            ber_val= curr_prbs_cnt / bits_transferred
            if(gLaneStats[gSlice][ln][8] == 'RDY'):
                print("%8.1e" % (ber_val) ),
            else:
                print skip,

    print line
    
    return myber
########################################### 
def rx_monitor_log (linecard_num=0,phy_num=0,slice_num=None,lane=None, rst=0, adapt_time=-1, header=1, logfile='rx_monitor_log.txt'): 
    global gLaneStats #[per Slice][per lane], [PrbsCount, PrbsCount-1,PrbsCount-2, PrbsRstTime, PrbsLastReadoutTime]
    if slice_num==None: 
        slice_num = gSlice
    post_fec_ber={}

   
    lanes = get_lane_list(lane)
    get_lane_mode(lanes)
 
    rx_monitor_capture(lane=lanes)
    
    log_file = open(logfile, 'a+')
    if header:
        ##### First Line of Header
        log_file.write("\nLc/Ph/Sl,Adapt,  BER"),
        for ln in lanes:  
            log_file.write(",%7s" %(lane_name_list[ln])),   # post-FEC BER            
        for ln in lanes:  
            log_file.write(",%7s" %(lane_name_list[ln])),   #  pre-FEC BER 
        for ln in lanes:  
            if(gEncodingMode[gSlice][ln][0].upper()== 'PAM4'):
                log_file.write(",%4s" %(lane_name_list[ln])),   # PAM4 Eye1
                log_file.write(",%4s" %(lane_name_list[ln])),   # PAM4 Eye2   
                log_file.write(",%4s" %(lane_name_list[ln])),   # PAM4 Eye3
            else:
                log_file.write(",%4s" %(lane_name_list[ln])),   # NRZ Eye
            
        ##### Second Line of Header
        log_file.write("\nLc/Ph/Sl, Time, Time"),
        for ln in lanes:  
            log_file.write(",Fec-BER"), # post-FEC BER         
        for ln in lanes:  
            log_file.write(",Raw-BER"), #  pre-FEC BER 
        for ln in lanes:  
            if(gEncodingMode[gSlice][ln][0].upper()== 'PAM4'):
                log_file.write(",Eye1"),    # PAM4 Eye1
                log_file.write(",Eye2" ),   # PAM4 Eye2   
                log_file.write(",Eye3" ),   # PAM4 Eye3
            else:
                log_file.write(", Eye"),    # NRZ Eye


    ##### Third Line is data, Linecard/Phy/Slice
    log_file.write("\n%2d/%2d/%2d" %(linecard_num,phy_num,slice_num)),   
    log_file.write(",%5.1f"%(adapt_time)),
    ber_time=gLaneStats[gSlice][lanes[0]][4] - gLaneStats[gSlice][lanes[0]][3]
    log_file.write(",%5.0f"% (ber_time) ),            # BER Collection time
    
    ##### Third Line is data, post-FEC BER
    for ln in lanes:
        if(gLaneStats[gSlice][ln][8] == 'RDY'):                
            curr_prbs_cnt = float(gLaneStats[gSlice][ln][10])
            prbs_accum_time = gLaneStats[gSlice][ln][4] - gLaneStats[gSlice][ln][3]
            data_rate = gEncodingMode[gSlice][ln][1]
            bits_transferred = float((prbs_accum_time*data_rate*pow(10,9)))       

            if curr_prbs_cnt==0 or bits_transferred==0: 
                ber_val = 0
                log_file.write(",%7d" % (ber_val) ), # post-FEC BER
            else:
                ber_val= curr_prbs_cnt / bits_transferred
                log_file.write(",%7.1e" % (ber_val) ),# post-FEC BER
            post_fec_ber[ln] = ber_val
        else: # Lane's PHY RDY = 0
            log_file.write(",      -"),  # post-FEC BER
            
    ##### Third Line is data, pre-FEC BER
    for ln in lanes:
        if(gLaneStats[gSlice][ln][8] == 'RDY'):             
            curr_prbs_cnt = float(gLaneStats[gSlice][ln][0])
            prbs_accum_time = gLaneStats[gSlice][ln][4] - gLaneStats[gSlice][ln][3]
            data_rate = gEncodingMode[gSlice][ln][1]
            bits_transferred = float((prbs_accum_time*data_rate*pow(10,9)))       
            if curr_prbs_cnt==0 or bits_transferred==0: 
                ber_val = 0
                log_file.write(",%7d" % (ber_val) ), # pre-FEC BER
            else:
                ber_val= curr_prbs_cnt / bits_transferred
                log_file.write(",%7.1e" % (ber_val) ), # pre-FEC BER
        else: # Lane's PHY RDY = 0
            log_file.write(",      -"),  # pre-FEC BER

    ##### Third Line is data, EYE MARGINS
    for ln in lanes:
        if(gLaneStats[gSlice][ln][8] == 'RDY'):
            if(gEncodingMode[gSlice][ln][0].upper()== 'PAM4'):
                log_file.write (",%4.0f"%(gLaneStats[gSlice][ln][5])),  # Eye1 
                log_file.write (",%4.0f"%(gLaneStats[gSlice][ln][6])),  # Eye2
                log_file.write (",%4.0f"%(gLaneStats[gSlice][ln][7])),  # Eye3
            else:
                log_file.write (",%4.0f"%(gLaneStats[gSlice][ln][5])),  # Eye
        else: # Lane's PHY RDY = 0
            if(gEncodingMode[gSlice][ln][0].upper()== 'PAM4'):
                log_file.write(",   -"),  # PAM4 Eye1
                log_file.write(",   -"),  # PAM4 Eye2   
                log_file.write(",   -"),  # PAM4 Eye3   
            else:
                log_file.write(",   -"),  # NRZ Eye
            

    if header:
        log_file.write("\n")
    log_file.close()
    return post_fec_ber , ber_time
########################################### 
def rx_monitor(lane=None, rst=0, lc=0,ph=0,sl=None, fec_thresh=15, print_en=1, returnon = 0):

    lanes = get_lane_list(lane)
        
    if (rst>0): 
        if rst>1:
            print("BER Accumulation Time: %1.1f sec"%(float(rst)))
        rx_monitor_clear(lane=lanes,fec_thresh=fec_thresh)
        if rst>1:
            time.sleep(float(rst-(len(lanes)*0.05)))
    else: # rst=0
        # in case any of the lanes' PRBS not cleared or FEC Analyzer not initialized, clear its Stats first
        for ln in lanes:
            if gLaneStats[gSlice][ln][3]==0 or gLaneStats[gSlice][ln][8] != 'RDY':
                rx_monitor_clear(lane=ln,fec_thresh=fec_thresh)
                #print("\nSlice %d Lane %s RX_MONITOR Initialized First!"%(gSlice,lane_name_list[ln])),
        
    rx_monitor_capture(lane=lane)
    if print_en:
        mydata = rx_monitor_print(lane=lane,lc=lc,ph=ph,sl=sl)
    if returnon ==1: return mydata
####################################################################    
def ber(lane=None, rst=0, t=None,fec_thresh=15):

    lanes = get_lane_list(lane)
        
    if (rst==1 or t!=None): 
        rx_monitor_clear(lane=lanes,fec_thresh=fec_thresh)
        #time.sleep(0.9)
    else: # rst=0
        # in case any of the lanes' PRBS not cleared or FEC Analyzer not initialized, clear its Stats first
        for ln in lanes:
            if gLaneStats[gSlice][ln][3]==0 or gLaneStats[gSlice][ln][8] != 'RDY':
                rx_monitor_clear(lane=ln,fec_thresh=fec_thresh)
                #print("\nSlice %d Lane %s RX_MONITOR Initialized First!"%(gSlice,lane_name_list[ln])),
        
    if (t!=None):
        time.sleep(t)
    rx_monitor_capture(lane=lane)
    result={}
    for ln in lanes:
        prbs_accum_time = gLaneStats[gSlice][ln][4] - gLaneStats[gSlice][ln][3]
        data_rate = gEncodingMode[gSlice][ln][1]
        bits_transferred = float((prbs_accum_time*data_rate*pow(10,9)))       
        
        prbs_err_cnt = float(gLaneStats[gSlice][ln][0])
        pre_fec_cnt  = float(gLaneStats[gSlice][ln][9])
        post_fec_cnt = float(gLaneStats[gSlice][ln][10])
        if(gLaneStats[gSlice][ln][8] != 'RDY'):
            prbs_ber = 1
            pre_fec_ber = 1
            post_fec_ber = 1
        else:
            #### PRBS BER
            if prbs_err_cnt==0 or bits_transferred==0: 
                prbs_ber = 0
            else:
                prbs_ber= prbs_err_cnt / bits_transferred
            #### Pre-FEC BER
            if pre_fec_cnt==0 or bits_transferred==0: 
                pre_fec_ber = 0
            else:
                pre_fec_ber= pre_fec_cnt / bits_transferred
            #### Post-FEC BER
            if post_fec_cnt==0 or bits_transferred==0: 
                post_fec_ber = 0
            else:
                post_fec_ber= post_fec_cnt / bits_transferred

        result[ln] = prbs_ber, post_fec_ber
    return result
####################################################################    
def ber_test(lane=None, rst=0, t=None, nrz_limit=0, pam4_limit=1e-6 ):

    pre_fec_ber_limit = 0
    post_fec_ber_limit = 0
    result={}
    lanes = get_lane_list(lane)
    ber_data = ber(lane, rst, t)
    for ln in lanes:        
        pre_fec_ber_limit = nrz_limit if lane_mode_list[ln].lower() == 'nrz' else pam4_limit
        if (ber_data[ln][0] > pre_fec_ber_limit) or (ber_data[ln][1] > post_fec_ber_limit):
                result[ln]= 'Fail'
        else:
            result[ln]= 'Pass'
    return result
####################################################################################################    
# copy_lane(source_lane, destination_lane)
#
# This function copies one lanes register to another lane, addr by addr
####################################################################################################    
def copy_lane(lane1, lane2):

    first_addr=0x000
    last_addr =0x1FF
     
    for addr in range(first_addr, last_addr+1, +1):
        val=rreg(addr,lane1)
        wreg(addr, val,lane2)   
        
    if lane2%2: # destination lane is an ODD lane
        set_bandgap(bg_val=7,lane=lane2)
    else:      # destination lane is an Even lane
        set_bandgap(bg_val=2,lane=lane2)

    lr(lane=lane2)
    print("\n....Slice %d Lane %s all registers copied to Lane %s "%(gSlice,lane_name_list[lane1],lane_name_list[lane2]))

####################################################################################################
# get_top_pll ()
#
# reads TOP-PLL registers and computes TOP-PLL Frequencies 
#
# returns all parameters for the specified top pll
#
####################################################################################################    
def get_top_pll ():

    top_pll_params = {}
    
    ext_ref_clk=156.25
    
    ## TOP PLL A (0x9500) & TOP PLL B (0x9600)
    # top_pll_en_refclk_addr = [0x9500, [7]]
    # top_pll_pu_addr = [0x9501, [2]]
    # top_pll_div4_addr = [0x9500, [6]]
    # top_pll_n_addr = [0x9507, [12,4]]
    # top_pll_lvcocap_addr = [0x9501, [12,6]] 
    # top_pll_refclk_div_addr = [0x9513, [15,7]]
    # top_pll_div2_addr = [0x9513, [6]]

    for pll_idx in [0,1]:
        div4_en     = rregBits(c.top_pll_div4_addr[0]+(pll_idx*0x100)       ,c.top_pll_div4_addr[1]   )
        div2_bypass = rregBits(c.top_pll_div2_addr[0]+(pll_idx*0x100)       ,c.top_pll_div2_addr[1]   )
        refclk_div  = rregBits(c.top_pll_refclk_div_addr[0]+(pll_idx*0x100) ,c.top_pll_refclk_div_addr[1])
        pll_n       = rregBits(c.top_pll_n_addr[0]+(pll_idx*0x100)          ,c.top_pll_n_addr[1]      )
        pll_cap     = rregBits(c.top_pll_lvcocap_addr[0]+(pll_idx*0x100)    ,c.top_pll_lvcocap_addr[1])
      
        div_by_4 = 1.0 if div4_en==0     else 4.0
        mul_by_2 = 1.0 if div2_bypass==1 else 2.0
       
        fvco = (ext_ref_clk * float(pll_n) *2.0 * mul_by_2) / (div_by_4 * float(refclk_div) * 8.0) # in MHz 
    
        top_pll_params[pll_idx] = fvco, pll_cap, pll_n, div4_en, refclk_div, div2_bypass, ext_ref_clk
        
    return top_pll_params    #data_rate, fvco, pll_cap, pll_n, div4, div2, ext_ref_clk
####################################################################################################
# 
# Top PLL Setup Configuration
####################################################################################################
def set_top_pll(pll_side='both', freq=195.3125):

    global gRefClkFreq; gRefClkFreq=195.3125
    
    if pll_side==None: pll_side='both'
    if type(pll_side)==int: 
        if pll_side==0: pll_side='A'
        else: pll_side='B'
    if type(pll_side)==str: pll_side=pll_side.upper()

   
    ##### If FW already loaded and pll_top_cal done, then save the cap values to restore them
    #if fw_loaded(print_en=0):
    top_pll_A_cap = rreg([0x9501,[12,6]])
    top_pll_B_cap = rreg([0x9601,[12,6]])

    ##### Top PLL B-side lanes
    if pll_side == 'A' or pll_side == 'BOTH':
        wreg(0x9500, 0x1aa0)
        wreg(0x9500, 0x0aa0) # [12]=0, do not bypass 1p7 regulator
        wreg(0x9501, 0x8a5b) # Top PLL A powered down while programming it, bit [2]=0
        wreg(0x9502, 0x03e6) # 0x03b6
        wreg(0x9503, 0x6d86)
        wreg(0x9504, 0x7180)
        wreg(0x9505, 0x9000)
        wreg(0x9506, 0x0000)
        wreg(0x9507, 0x0500)
        wreg(0x950a, 0x4040)
        wreg(0x950b, 0x0000)
        wreg(0x950d, 0x0000)
        wreg(0x9510, 0xb670)
        wreg(0x9511, 0x0000)
        wreg(0x9512, 0x7de8)
        wreg(0x9513, 0x0840)
        
        wreg(0x9512, 0xfde8)
        wreg(0x9501, 0x8a5f) # Top PLL A on after programming it, bit [2]=1
        
        wreg(0x9501, 0x89df) # default top_pll_A_cap = 0x27        
        if fw_loaded(print_en=0):      # if FW loaded, restore cal'ed value for top_pll_A_cap
            wreg([0x9501,[12,6]], top_pll_A_cap)

        if type(freq)==str and freq.upper()=='OFF':
            wregBits(0x9501, [2], 0)
            
    ##### Top PLL B-side lanes   
    if pll_side == 'B' or pll_side == 'BOTH':   
        wreg(0x9600, 0x1aa0)
        wreg(0x9600, 0x0aa0) # [12]=0, do not bypass 1p7 regulator
        wreg(0x9601, 0x8a5b) # Top PLL B powered down while programming it, bit [2]=0
        wreg(0x9602, 0x03e6) # 0x03b6
        wreg(0x9603, 0x6d86)
        wreg(0x9604, 0x7180)
        wreg(0x9605, 0x9000)
        wreg(0x9606, 0x0000)
        wreg(0x9607, 0x0500)
        wreg(0x960a, 0x4040)
        wreg(0x960b, 0x0000)
        wreg(0x960d, 0x0000)
        wreg(0x9610, 0xb670)
        wreg(0x9611, 0x0000)
        wreg(0x9612, 0x7de8)
        wreg(0x9613, 0x0840)
        
        wreg(0x9612, 0xfde8)
        wreg(0x9601, 0x8a5f) # Top PLL A powered up after programming it, bit [2]=1
        
        wreg(0x9601, 0x89df) # default top_pll_B_cap = 0x27        
        if fw_loaded(print_en=0):      # if FW loaded, restore cal'ed value for top_pll_B_cap
            wreg([0x9601,[12,6]], top_pll_B_cap)
            
        if type(freq)==str and freq.upper()=='OFF':
            wregBits(0x9601, [2], 0)
            
####################################################################################################
def set_top_pll_bypass(pll_side='both', freq=195.3125):

    global gRefClkFreq; gRefClkFreq=195.3125
    
    if pll_side==None: pll_side='both'
    if type(pll_side)==int: 
        if pll_side==0: pll_side='A'
        else: pll_side='B'
    if type(pll_side)==str: pll_side=pll_side.upper()
    
    ##### If FW already loaded and pll_top_cal done, then save the cap values to restore them
    if fw_loaded(print_en=0):
        top_pll_A_cap = rreg([0x9501,[12,6]])
        top_pll_B_cap = rreg([0x9601,[12,6]])

    ##### Top PLL B-side lanes
    if pll_side == 'A' or pll_side == 'BOTH':
        wreg(0x9500, 0x1a20) # Bypass Top PLL A
        wreg(0x9500, 0x0a20) # [12]=0, do not bypass 1p7 regulator
        #wreg(0x9500, 0x1aa0)
        #wreg(0x9500, 0x0aa0) # [12]=0, do not bypass 1p7 regulator
        wreg(0x9501, 0x8a5b) # Top PLL A poewred down while programming it, bit [2]=0
        wreg(0x9502, 0x03e6) # 0x03b6
        wreg(0x9503, 0x6d86)
        wreg(0x9504, 0x7180)
        wreg(0x9505, 0x9000)
        wreg(0x9506, 0x0000)
        wreg(0x9507, 0x0500)
        wreg(0x950a, 0x4040)
        wreg(0x950b, 0x0000)
        wreg(0x950d, 0x0000)
        wreg(0x9510, 0xb670)
        wreg(0x9511, 0x0000)
        wreg(0x9512, 0x7de8)
        wreg(0x9513, 0x0840)
        
        wreg(0x9512, 0xfde8)
        wreg(0x9501, 0x8a5f) # Top PLL A on after programming it, bit [2]=1
        
        wreg(0x9501, 0x89df) # default top_pll_A_cap = 0x27        
        if fw_loaded(print_en=0):      # if FW loaded, restore cal'ed value for top_pll_A_cap
            wreg([0x9501,[12,6]], top_pll_A_cap)

        if type(freq)==str and freq.upper()=='OFF':
            wregBits(0x9501, [2], 0)
            
    ##### Top PLL B-side lanes   
    if pll_side == 'B' or pll_side == 'BOTH':
        wreg(0x9600, 0x1a20) 
        wreg(0x9600, 0x0a20) # [12]=0, do not bypass 1p7 regulator    
        #wreg(0x9600, 0x1aa0)
        #wreg(0x9600, 0x0aa0) # [12]=0, do not bypass 1p7 regulator
        wreg(0x9601, 0x8a5b) # Top PLL B powered down while programming it, bit [2]=0
        wreg(0x9602, 0x03e6) # 0x03b6
        wreg(0x9603, 0x6d86)
        wreg(0x9604, 0x7180)
        wreg(0x9605, 0x9000)
        wreg(0x9606, 0x0000)
        wreg(0x9607, 0x0500)
        wreg(0x960a, 0x4040)
        wreg(0x960b, 0x0000)
        wreg(0x960d, 0x0000)
        wreg(0x9610, 0xb670)
        wreg(0x9611, 0x0000)
        wreg(0x9612, 0x7de8)
        wreg(0x9613, 0x0840)
        
        wreg(0x9612, 0xfde8)
        wreg(0x9601, 0x8a5f) # Top PLL A powered up after programming it, bit [2]=1
        
        wreg(0x9601, 0x89df) # default top_pll_B_cap = 0x27        
        if fw_loaded(print_en=0):      # if FW loaded, restore cal'ed value for top_pll_B_cap
            wreg([0x9601,[12,6]], top_pll_B_cap)
            
        if type(freq)==str and freq.upper()=='OFF':
            wregBits(0x9601, [2], 0)
    
  
####################################################################################################
# 
# Top PLL Setup Configuration
####################################################################################################
def set_bandgap(bg_val=None, lane=None):
   
    lanes = get_lane_list(lane)

    if bg_val!=None:
        for lane in lanes:
            if type(bg_val) == int:
                bg_val = max(0, min(bg_val, 7))
                wregBits(0x0ff,[12],1,lane) # power up TX BG
                wregBits(0x1ff,[12],1,lane) # power up RX BG
                wregBits(0x0ff,[15,13],bg_val,lane)
                wregBits(0x1ff,[15,13],bg_val,lane)
            elif bg_val.upper()=='OFF':     # power down BG
                wregBits(0x0ff,[12],0,lane)
                wregBits(0x1ff,[12],0,lane)
            else: #if bg_val.upper()=='ON': # power up BG, leave BG value as is
                wregBits(0x0ff,[12],1,lane)
                wregBits(0x1ff,[12],1,lane)
                
    if bg_val==None:
        tx_bg_pu  =[] 
        rx_bg_pu  =[] 
        tx_bg_val =[]                
        rx_bg_val =[]                
        for lane in range(0,len(lane_name_list)):
            tx_bg_pu.append (rreg([0x0ff,   [12]],lane)) # TX BG powered up/down?
            rx_bg_pu.append (rreg([0x1ff,   [12]],lane)) # RX BG powered up/down?
            tx_bg_val.append(rreg([0x0ff,[15,13]],lane)) # TX BG value
            rx_bg_val.append(rreg([0x1ff,[15,13]],lane)) # RX BG value

        print "Lanes    :",lanes
        print "TX BG PU :",tx_bg_pu
        print "RX BG PU :",rx_bg_pu
        print "TX BG Val:",tx_bg_val
        print "RX BG Val:",rx_bg_val

####################################################################################################
# Program Clock Output Test Points (CLKOUT0 +/- DIFF, CLKOUT1 single-ended or CLKOUT2 single-ended)
#
#
####################################################################################################
def set_clock_out(clkout='???', clkdiv=32, lane=0,print_en=0):
   
    CLKOUT_REG = [0x98D5,0x98D6,0x98D7,0x98D8,0x98D9]
    clkout_div_list=[32,64,128,256,512,1024,2048,4096]
    if clkdiv in clkout_div_list:
        div_index = clkout_div_list.index(clkdiv)
    else:
        div_index=0
        clkdiv=clkout_div_list[div_index]
    
    ###### CLKOUT0 Differential output pins
    if clkout.lower() == '0': 
        wreg([CLKOUT_REG[0],[ 3, 0]],lane)      # select target lane's clock to send to clock out 0
        wreg([CLKOUT_REG[1],[   12]],1)         # enable clock out 0 mux
        wreg([CLKOUT_REG[1],[ 2, 0]],div_index) # select clock out 0 divider
        wreg([CLKOUT_REG[3],[14,13]],3)         # enable clock out main driver
        wreg([CLKOUT_REG[3],[11, 8]],0xF)       # enable clock out 0 driver
        wreg([CLKOUT_REG[4],[13, 0]],0x3FFF)    # power up bandgap for clock out driver
        pll_params = get_lane_pll(lane)
        data_rate=pll_params[lane][0][0]
        fvco=pll_params[lane][0][1]
        fclkout=float(1000*fvco/float(clkdiv))        
        print("\n...Turned On CLKOUT0 Diff pins, PLL Lane: %d, ClkOutDivider: %d, BitRate: %2.5f Gbps, PLL: %2.5f GHz, FclkOut0: %2.5f MHz"%(lane,clkdiv,data_rate,fvco,fclkout))
        
    ###### CLKOUT1 single-ended output pin
    elif clkout.lower() == '1': 
        wreg([CLKOUT_REG[0],[ 7, 4]],lane)      # select target lane's clock to send to clock out 1
        wreg([CLKOUT_REG[1],[   13]],1)         # enable clock out 1 mux
        wreg([CLKOUT_REG[1],[ 5, 3]],div_index) # select clock out 1 divider
        wreg([CLKOUT_REG[3],[14,13]],3)         # enable clock out main driver
        wreg([CLKOUT_REG[3],[ 7, 4]],0xF)       # enable clock out 1 driver
        wreg([CLKOUT_REG[4],[13, 0]],0x3FFF)    # power up bandgap for clock out driver
        pll_params = get_lane_pll(lane)
        data_rate=pll_params[lane][0][0]
        fvco=pll_params[lane][0][1]
        fclkout=float(1000*fvco/float(clkdiv))        
        print("\n...Turned On CLKOUT1 pin, PLL Lane: %d, ClkOutDivider: %d, BitRate: %2.5f Gbps, PLL: %2.5f GHz, FclkOut1: %2.5f MHz"%(lane,clkdiv,data_rate,fvco,fclkout))
    ###### CLKOUT2 single-ended output pin
    elif clkout.lower() == '2': 
        wreg([CLKOUT_REG[0],[11, 8]],lane)      # select target lane's clock to send to clock out 2
        wreg([CLKOUT_REG[1],[   14]],1)         # enable clock out 2 mux
        wreg([CLKOUT_REG[1],[ 8, 6]],div_index) # select clock out 2 divider
        wreg([CLKOUT_REG[3],[14,13]],3)         # enable clock out main driver
        wreg([CLKOUT_REG[3],[ 3, 0]],0xF)       # enable clock out 2 driver
        wreg([CLKOUT_REG[4],[13, 0]],0x3FFF)    # power up bandgap for clock out driver
        pll_params = get_lane_pll(lane)
        data_rate=pll_params[lane][0][0]
        fvco=pll_params[lane][0][1]
        fclkout=float(1000*fvco/float(clkdiv))        
        print("\n...Turned On CLKOUT2 pin, PLL Lane: %d, ClkOutDivider: %d, BitRate: %2.5f Gbps, PLL: %2.5f GHz, FclkOut2: %2.5f MHz"%(lane,clkdiv,data_rate,fvco,fclkout))
    elif clkout.lower() == 'off': 
        wreg(CLKOUT_REG[0],0x0000)             # back to the default lane 0 for all clock out pins
        wreg(CLKOUT_REG[1],0x0000)             # disable all clock out mux'es
        wreg(CLKOUT_REG[2],0x0000)             # default 
        wreg(CLKOUT_REG[3],0x0000)             # disable clock out main driver
        wreg(CLKOUT_REG[4],0x0000)             # power down bandgap for clock out driver
        print("\n...Turned Off ALL Three Clock Output Pins!")
    else:
        print("\n...Usage:")
        print("\n   set_clock_out (clkout='0', clkdiv=32, lane=0)"),
        print("\n   set_clock_out (clkout='1', clkdiv=32, lane=0)"),
        print("\n   set_clock_out (clkout='2', clkdiv=32, lane=0)"),
        print("\n   set_clock_out ('OFF') ")
        print("\n...Parameters:")
        print("\n   clkout : selects clock-out pin to program"),
        print("\n           '0'   :Outout to differential clock-out pins CKOP/CKON "),
        print("\n           '1'   :Output to Single-ended clock-out pin  CLK_OUT1  "),
        print("\n           '2'   :Output to Single-ended clock-out pin  CLK_OUT2  "),
        print("\n           'off' :Turn OFF all three clock-out pins")
        print("\n   clkdiv : selects clock-out divider (from target lane's PLL) "),
        print("\n            choices: 32,64,128,256,1024,2048 or 4096 ")
        print("\n   lane   : selects target lane's PLL as source for clock-out pin"),
        print("\n            choices: : 0 to 15")
    
    if print_en: reg([0x98D5,0x98D6,0x98D7,0x98D8,0x98D9])
####################################################################################################
# Read On-Chip Temperature Sensor
# 
####################################################################################################
def temp_sensor_fast(sel=0):
    
    base = 0xb000
    Yds = 237.7
    Kds = 79.925
    addr= [base+0x39,base+0x3a,base+0x3b,base+0x3c]
    MdioWr(base+0x37, 0x2d) # set write ack. Do this to make the first read correct
    MdioWr(base+0x37, 0x0d) # set read with ack
    #time.sleep(1)
    rdy = MdioRd(base+0x38)&0x01
    time1=time.time()
    time2=time.time()
    while(rdy==0): # wait for rdy
        rdy = MdioRd(base+0x38)&0x01
        time2=time.time()
        if (time2-time1)>=2:
            print 'TSensor Read-back Timed Out!'
            break

    value = MdioRd(addr[sel])
    realVal = value*Yds/4096-Kds
    #print('TempSensor: %3.1f C'%(realVal)),
    #print('tempsensor%d: %d,realVal:%f'%(sel,value,realVal))
    MdioWr(base+0x37, 0x2d) #write ack
    return realVal
####################################################################################################
# Read On-Chip Temperature Sensor
# 
####################################################################################################
def temp_sensor():
    temp_sensor_start()    
    value1=temp_sensor_read()
    print('Device TempSensor: %3.1f C'%(value1)),

####################################################################################################
# Set up On-Chip Temperature Sensor (to be read later)
# 
####################################################################################################
def temp_sensor_start():
    base = 0xB000 
    #MdioWr(0x81f7,0x4d80) #set tst0 value
    MdioWr(base+0x3e, 0x10d) #set clock
    time.sleep(0.1)
    MdioWr(base+0x37, 0x0) #reset sensor
    time.sleep(0.1)
    MdioWr(base+0x37, 0xc) #set no ack      
####################################################################################################
# Read back On-Chip Temperature Sensor (after it's been set up aready)
# 
####################################################################################################
def temp_sensor_read():
    base = 0xB000 
    sense_addr= base+0x39
    Yds = 237.7
    Kds = 79.925
    
    rdy = MdioRd(base+0x38)>>8 
    while(rdy==0): # wait for rdy
        rdy = MdioRd(base+0x38)>>8 
    value1 = MdioRd(sense_addr)*Yds/4096-Kds
    #print('%3.1f C'%(value1)),
    return value1
####################################################################################################
# 
# Initialize AGC Gain1 
####################################################################################################
def dc_gain_search(lane=None,target_dac_val=9):
    lanes = get_lane_list(lane)

    Slice=gSlice
    #target_dac_val = 9
    agc_gain1_mult = 5
    agc_gain2_mult = 4
    
    for ln in lanes:
        lr(lane=ln) 
        time.sleep(1)
        dac_val0 = rreg(c.rx_dac_addr, lane=ln) #dac(lane=ln)[ln] # read DAC value
        dac_val2 = dac_val0
        agc1,agc2,fgain1,fgain2 = dc_gain(lane=ln)[ln]
        new_agc1 = agc1
        new_agc2 = agc2
        while dac_val2 != target_dac_val:
            gain_delta = (target_dac_val - dac_val2) * agc_gain1_mult # calculate the change in agcgain1
            [agc1,agc2,fgain1,fgain2] = dc_gain(lane=ln)[ln] # read back the current DC Gain
            new_agc1 = agc1+gain_delta
            new_agc2 = agc2
            if new_agc1 > 127: new_agc1=127
            if new_agc1 <= 1: 
                new_agc1=1 # absolute minimum AGC1 is 1
                lr(lane=ln); time.sleep(.5)
                dac_val2 = rreg(c.rx_dac_addr, lane=ln)
                gain_delta = (target_dac_val - dac_val2) * agc_gain2_mult
                new_agc2 = agc2+gain_delta
                if new_agc2 <= 10: new_agc2=10 # absolute minimum AGC2 is 10
                if new_agc2 > 31: new_agc2 = 31
            dc_gain(agcgain1=new_agc1, agcgain2=new_agc2, lane=ln)
            lr(lane=ln); time.sleep(.5)
            dac_val2 = rreg(c.rx_dac_addr, lane=ln)
            if (new_agc1 == 127) & (new_agc2 == 31): break
            if (new_agc1 == 1) & (new_agc2 == 10): break
            
        dc_gain(agcgain1=new_agc1, agcgain2=new_agc2, lane=ln)
        lr(lane=ln) 
        time.sleep(.1)
        dac_val1 = rreg(c.rx_dac_addr, lane=ln) #dac(lane=ln)[ln] # read DAC value
        print ("\nSlice %d Lane %s (%s) DC-Gain Adjustment: OrigDAC: %d, NewDAC: %d, AGCGain1= %d->%d , AGCGain2= %d->%d"%(Slice, lane_name_list[ln],gEncodingMode[gSlice][ln][0].upper(),dac_val0,dac_val1,agc1, new_agc1, agc2,new_agc2)),
        
    return new_agc1, new_agc2
####################################################################################################
# 
# Initialize AGC Gain1 
####################################################################################################
def dc_gain_search_nrz(lane=None,target_dac_val=14):
    lanes = get_lane_list(lane)

    #target_dac_val = 9
    agc_gain1_mult = 5
    agc_gain2_mult = 4
    c=NrzReg
    
    for ln in lanes:
        lr(lane=ln) 
        time.sleep(1)
        dac_val0 = rreg(c.rx_dac_addr, lane=ln) #dac(lane=ln)[ln] # read DAC value
        dac_val2 = dac_val0
        agc1,agc2,fgain1,fgain2 = dc_gain(lane=ln)[ln]
        new_agc1 = agc1
        new_agc2 = agc2
        while dac_val2 != target_dac_val:
            gain_delta = (target_dac_val - dac_val2) * agc_gain1_mult # calculate the change in agcgain1
            [agc1,agc2,fgain1,fgain2] = dc_gain(lane=ln)[ln] # read back the current DC Gain
            new_agc1 = agc1+gain_delta
            new_agc2 = agc2
            if new_agc1 > 127: new_agc1=127
            if new_agc1 <= 1: 
                new_agc1=1 # absolute minimum AGC1 is 1
                lr(lane=ln); time.sleep(.5)
                dac_val2 = rreg(c.rx_dac_addr, lane=ln)
                gain_delta = (target_dac_val - dac_val2) * agc_gain2_mult
                new_agc2 = agc2+gain_delta
                if new_agc2 <= 10: new_agc2=10 # absolute minimum AGC2 is 10
                if new_agc2 > 31: new_agc2 = 31
            dc_gain(agcgain1=new_agc1, agcgain2=new_agc2, lane=ln)
            lr(lane=ln); time.sleep(.5)
            dac_val2 = rreg(c.rx_dac_addr, lane=ln)
            if (new_agc1 == 127) & (new_agc2 == 31): break
            if (new_agc1 == 1) & (new_agc2 == 10): break
            
        dc_gain(agcgain1=new_agc1, agcgain2=new_agc2, lane=ln)
        lr(lane=ln) 
        time.sleep(.1)
        dac_val1 = rreg(c.rx_dac_addr, lane=ln) #dac(lane=ln)[ln] # read DAC value
        print ("\nSlice %d Lane %s (%s) DC-Gain Adjustment: OrigDAC: %d, NewDAC: %d, AGCGain1= %d->%d , AGCGain2= %d->%d"%(gSlice, lane_name_list[ln],gEncodingMode[gSlice][ln][0].upper(),dac_val0,dac_val1,agc1, new_agc1, agc2,new_agc2)),
        
    return new_agc1, new_agc2
####################################################################################################    
# background_cal()  # returns status of current gLane
# background_cal(lane=11)  # returns status of current gLane
# background_cal('en', lane=11)  #  enable for lane 11 (B0)
# background_cal('dis',lane=11)  # disable for lane 11 (B0)
# background_cal()  # returns status of current gLane
# background_cal()  # returns status of current gLane
####################################################################################################    
def background_cal (enable=None,lane=None,print_en=1):

    lanes = get_lane_list(lane)

    get_lane_mode(lanes)
    background_cal_status=[]
    lane_cntr=0
    for ln in lanes:
        if gEncodingMode[gSlice][ln][0]=='pam4':  # Lane is in PAM4 mode
            c=Pam4Reg
        else:
            print"\nSlice %d Lane %s (NRZ) Background Cal: ***>> Lane is in NRZ Mode (No Background Cal!)\n"%(gSlice,lane_name_list[lane]),
            return

        # Asked to enable or disable background cal
        if enable!=None: 
            #### Clear BP2, Toggle BP1 and Continue to BP1 at state 0x12
            bp2(0,0x0,lane=ln)
            bp1(0,0x0,lane=ln)
            sm_cont_01(lane=ln)
            bp1(1,0x12,lane=ln) 
            wait_for_bp1_timeout=0
            # Wait for BP1 state 0x12 to be reached
            while True:
                if bp1(lane=ln)[ln][-1]: #BP1 state 0x12 is reached
                    break
                else:
                    wait_for_bp1_timeout+=1
                    if wait_for_bp1_timeout>5000:
                        if(print_en):print"\nSlice %d Lane %s (PAM4) Background Cal: ***>> Timed out waiting for BP1\n"%(gSlice,lane_name_list[ln]),
                        #save_setup('background_cal_timeout.txt')
                        break

            if enable=='en' or enable==1: # asked to enable background cal
                wreg ([0x4, [0]],  1, ln)
                wreg (0x077, 0x4e5c, ln)
                wreg (0x078, 0xe080, ln)
                wreg (c.rx_iter_s6_addr, 6, ln)
                wreg (c.rx_mu_ow_addr,   3, ln)
            else: # asked to disable background cal
                wreg (c.rx_iter_s6_addr, 1, ln)
                wreg (c.rx_mu_ow_addr,   0, ln)        
                #wreg (c.rx_iter_s6_addr, 1, ln)
                #wreg (c.rx_mu_ow_addr,   0, ln)
                
            bp1(0,0x12,lane=ln) # clear BP1
            sm_cont_01(lane=ln) # toggle SM Continue
            
        else:
            print_en=1
            
        # return Status of background cal for this lane
        if(print_en):print"\nSlice %d Lane %s (PAM4) Background Cal: "%(gSlice,lane_name_list[ln]),

        background_cal_status.append(rreg(c.rx_mu_ow_addr,ln) & 0x01)
        if background_cal_status[lane_cntr] == 0:
            if(print_en):print ("OFF")
        else:
            if(print_en):print ("ON")
        lane_cntr+=1 # next lane, if applicable
    
    return background_cal_status
        
####################################################################################################
# 
# lane_reset (used only when FW is NOT loaded)
####################################################################################################
def lr_no_fw(lane=None):

    lanes = get_lane_list(lane)
    get_lane_mode(lanes) # update the Encoding modes of all lanes for this Slice
    
    for lane in lanes:
        if gEncodingMode[gSlice][lane][0] == 'pam4': # Lane is in PAM4 mode
            c=Pam4Reg

            #print"\nSlice %d Lane %s (PAM4) is Reset"%(gSlice,lane_name_list[lane]),
            #wreg(0x002,0xc000,lane) # final cntr target
            super_cal = rreg(c.rx_mu_ow_addr,lane)
            if super_cal != 0:
                wreg(c.rx_mu_owen_addr,1,lane) # super-cal disable        
                wreg(c.rx_mu_ow_addr,0,lane) # super-cal disable
            updn = rreg(c.rx_theta_update_mode_addr,lane)
            if updn != 0:
                wreg(c.rx_theta_update_mode_addr, 0, lane)
            blc = rreg([0x07b,[8,6]],lane) # BLC?
            wreg([0x07b,[8,6]],0,lane) # BLC off 
            wreg(c.rx_theta_update_mode_addr,0,lane) # up/dn mode disable - rajan        
            
            wreg(c.rx_lane_rst_addr, 0x1, lane)
            time.sleep(.050)        
            wreg(c.rx_lane_rst_addr, 0x0, lane)
            
            # Before exiting Lane Reset, Restore parameters
            if updn != 0: wreg(c.rx_theta_update_mode_addr,updn,lane) # up/dn mode enable - rajan        
            wreg(c.rx_mu_owen_addr,1,lane) # super-cal enable      
            if super_cal != 0: wreg(c.rx_mu_ow_addr,super_cal,lane) # super-cal enable        
            wreg([0x07b,[8,6]],blc,lane) # BLC restored
            
        else: # Lane is in NRZ mode
            c=NrzReg
            #print"\nSlice %d Lane %s (NRZ) is Reset"%(gSlice,lane_name_list[lane]),
            wreg([0x07b,[8,6]],0,lane) # BLC off
            wreg(c.rx_cntr_target_addr, 0x100, lane)            
            wreg(c.rx_lane_rst_addr, 0x1, lane)
            time.sleep(.050)        
            wreg(c.rx_lane_rst_addr, 0x0, lane)
            wreg(c.rx_cntr_target_addr, 0x002, lane)            

    # Clear lane statistics, used in rx_monitor()
    #rx_monitor_clear(lane=lanes,fec_thresh=fec_thresh) 
    global gLaneStats #lane statistics, used in rx_monitor()
    for ln in lanes:  gLaneStats[gSlice][ln][3]=0 
####################################################################################################
# 
# lane_reset (used when FW is loaded)
####################################################################################################
def lr(lane=None):

    lanes = get_lane_list(lane)
    get_lane_mode(lanes) # update the Encoding modes of all lanes for this Slice
    
    for lane in lanes:
        c = Pam4Reg if gEncodingMode[gSlice][lane][0] == 'pam4' else NrzReg  # Lane is in PAM4 or NRZ mode?        
        wreg(c.rx_lane_rst_addr, 0x1, lane)
    time.sleep(.050)        
    for lane in lanes:
        c = Pam4Reg if gEncodingMode[gSlice][lane][0] == 'pam4' else NrzReg  # Lane is in PAM4 or NRZ mode?        
        wreg(c.rx_lane_rst_addr, 0x0, lane)
            
    global gLaneStats #lane statistics, used in rx_monitor()
    for ln in lanes:  gLaneStats[gSlice][ln][3]=0 
####################################################################################################
# 
# 
####################################################################################################
def lane_reset_fast (lane=None, wait=0.02):
    lanes = get_lane_list(lane)

    Slice=gSlice
    get_lane_mode(lanes) # update the Encoding modes of all lanes for this Slice
    
    for lane in lanes:
        if gEncodingMode[gSlice][lane][0]=='pam4':  # Lane is in PAM4 mode
            c=Pam4Reg
            #print"Lane %d is PAM4"%lane
            #wreg(0x002,0xc000,lane) # final cntr target
            wreg(c.rx_mu_owen_addr,1,lane) # super-cal disable        
            wreg(c.rx_mu_ow_addr,0,lane) # super-cal disable
            blc = rreg([0x07b,[8,6]],lane) # BLC?
            wreg([0x07b,[8,6]],0,lane) # BLC off 
            wreg(c.rx_theta_update_mode_addr,0,lane) # up/dn mode disable - rajan        
            
            wreg(c.rx_lane_rst_addr, 0x1, lane)
            
            wreg(c.rx_acal_start_owen_addr,1, lane)
            wreg(c.rx_acal_start_ow_addr,  0, lane)
            time.sleep(wait)
            wreg(c.rx_acal_done_owen_addr, 1, lane)
            wreg(c.rx_acal_done_ow_addr,   0, lane)
            
            wreg(c.rx_lane_rst_addr, 0x0, lane)
            
            wreg(c.rx_acal_done_owen_addr, 0, lane)
            wreg(c.rx_acal_start_owen_addr,0, lane)
            
            # Before exiting Lane Reset, Restore parameters
            wreg(c.rx_theta_update_mode_addr,7,lane) # up/dn mode enable - rajan        
            wreg(c.rx_mu_owen_addr,1,lane) # super-cal enable      
            wreg(c.rx_mu_ow_addr,3,lane) # super-cal enable        
            wreg([0x07b,[8,6]],blc,lane) # BLC restored
            
        else: # Lane is in NRZ mode
            c=NrzReg
            #print"Lane %d is NRZ"%lane
            wreg([0x07b,[8,6]],0,lane) # BLC off 
            wreg(c.rx_lane_rst_addr, 0x1, lane)
            
            wreg(c.rx_acal_start_owen_addr,1, lane)
            wreg(c.rx_acal_start_ow_addr,  0, lane)
            time.sleep(wait)
            wreg(c.rx_acal_done_owen_addr, 1, lane)
            wreg(c.rx_acal_done_ow_addr,   0, lane)
            
            wreg(c.rx_lane_rst_addr, 0x0, lane)
            
            wreg(c.rx_acal_done_owen_addr, 0, lane)
            wreg(c.rx_acal_start_owen_addr,0, lane)
                       
    
####################################################################################################
# 
# DC_GAIN 
####################################################################################################
def dc_gain2(ctle_gain1=None, ctle_gain2=None, ffe_gain1=None, ffe_gain2=None, lane=None):
    
    if ctle_gain1=='?':
        print("\n ***> Usage: dc_gain(ctle_gain1, ffe_gain1, ctle_gain2, ffe_gain2, lane#)"),
        ctle_gain1=None; ctle_gain2=None; ffe_gain1=None;  ffe_gain2=None; lane=None
        print("\n ***> Current settings shown below:")

    if lane==None: lane=gLane
    if type(lane)==int:     lanes=[lane]
    elif type(lane)==list:  lanes=list(lane)
    elif type(lane)==str and lane.upper()=='ALL': 
        lanes=range(0,len(lane_name_list))
        
    for lane in lanes:
        ## Convert from binary to Gray and write dc gain to registers
        if ctle_gain1 !=None:
            ctle_gain1_gray = Bin_Gray(ctle_gain1)
            wreg(c.rx_agcgain1_addr, ctle_gain1_gray, lane)
                
        if ctle_gain2 !=None:
            ctle_gain2_gray = Bin_Gray(ctle_gain2)
            wreg(c.rx_agcgain2_addr, ctle_gain2_gray, lane)
                
        if ffe_gain1 !=None:
            ffe_gain1_gray = Bin_Gray(ffe_gain1)
            wreg(c.rx_ffe_sf_msb_addr, ffe_gain1_gray, lane)
            
        if ffe_gain2 !=None:
            ffe_gain2_gray = Bin_Gray(ffe_gain2)
            wreg(c.rx_ffe_sf_lsb_addr, ffe_gain2_gray, lane)
        
        ##read DC gain from register and convert to binary from gray code
        ctle_gain1_bin = Gray_Bin (rreg(c.rx_agcgain1_addr, lane))
        ctle_gain2_bin = Gray_Bin (rreg(c.rx_agcgain2_addr, lane))
        ffe_gain1_bin  = Gray_Bin (rreg(c.rx_ffe_sf_msb_addr, lane))
        ffe_gain2_bin  = Gray_Bin (rreg(c.rx_ffe_sf_lsb_addr, lane))
    
    return ctle_gain1_bin, ctle_gain2_bin, ffe_gain1_bin, ffe_gain2_bin
####################################################################################################
# 
# set or get CTLE PEAKING index selection 
####################################################################################################
def ctle2(val = None, lane = None):
    lanes = get_lane_list(lane)
        
    for lane in lanes:
        if val != None:
            wreg(c.rx_agc_ow_addr, val, lane)
            wreg(c.rx_agc_owen_addr, 0x1, lane)
            
        val = rreg(c.rx_agc_ow_addr, lane)
        
    return val
####################################################################################################
# 
# set or get CTLE PEAKING 1/2 settings
####################################################################################################
def ctle_map2(sel = None, val1 = None, val2 = None, lane = None):

    lanes = get_lane_list(lane)

    for lane in lanes:
        if not None in (sel, val1, val2):
            if sel != 2 and sel != 5:
                val = (val1<<3) + val2
                if sel == 0:
                    map0 = (rreg(c.rx_agc_map0_addr, lane) | 0xfc00) & (val<<10 | 0x3ff) 
                    wreg(c.rx_agc_map0_addr, map0, lane)
                elif sel == 1:
                    map0 = (rreg(c.rx_agc_map0_addr, lane) | 0x03f0) & (val<<4 | 0xfc0f) 
                    wreg(c.rx_agc_map0_addr, map0, lane)
                elif sel == 3:
                    map1 = (rreg(c.rx_agc_map1_addr, lane) | 0x3f00) & (val<<8 | 0xc0ff) 
                    wreg(c.rx_agc_map1_addr, map1, lane)
                elif sel == 4:
                    map1 = (rreg(c.rx_agc_map1_addr, lane) | 0x00fc) & (val<<2 | 0xff03) 
                    wreg(c.rx_agc_map1_addr, map1, lane)
                elif sel == 6:
                    map2 = (rreg(c.rx_agc_map2_addr, lane) | 0x0fc0) & (val<<6 | 0xf03f) 
                    wreg(c.rx_agc_map2_addr, map2, lane)
                elif sel == 7:
                    map2 = (rreg(c.rx_agc_map2_addr, lane) | 0x003f) & (val | 0xffc0) 
                    wreg(c.rx_agc_map2_addr, map2, lane)
            elif sel == 2:
                val = (val1<<1) + (val2 & 0x4)
                map0 = (rreg(c.rx_agc_map0_addr, lane) | 0x000f) & (val | 0xfff0)
                wreg(c.rx_agc_map0_addr, map0, lane)
                val = val2 & 0x3
                map1 = (rreg(c.rx_agc_map1_addr, lane) | 0xc000) & (val | 0x3fff)
                wreg(c.rx_agc_map1_addr, map1, lane)
            elif sel == 5:
                val = val1 & 0x6
                map1 = (rreg(c.rx_agc_map1_addr, lane) | 0x3) & (val | 0xfffc)
                wreg(c.rx_agc_map1_addr, map1, lane)
                val = ((val1 & 0x1)<<3) + val2
                map2 = (rreg(c.rx_agc_map2_addr, lane) | 0xf) & (val | 0x0fff)
                wreg(c.rx_agc_map2_addr, map2, lane)
                
        # Read CTLE Peaking table values for each/this lane
        map0 = rreg(c.rx_agc_map0_addr, lane)
        map1 = rreg(c.rx_agc_map1_addr, lane)
        map2 = rreg(c.rx_agc_map2_addr, lane)
        agc = { 0: [map0>>13, (map0>>10) & 0x7],
                1: [(map0>>7) & 0x7, (map0>>4) & 0x7],
                2: [(map0>>1) & 0x7, ((map0 & 0x1)<<2) + (map1>>14)],
                3: [(map1>>11) & 0x7, (map1>>8) & 0x7],
                4: [(map1>>5) & 0x7, (map1>>2) & 0x7],
                5: [((map1 & 0x3)<<1) + (map2>>15), (map2>>12) & 0x7],
                6: [(map2>>9) & 0x7, (map2>>6) & 0x7],
                7: [(map2>>3) & 0x7, map2 & 0x7]
                }
                
    if sel == None:
        for key in agc.keys():
            print key, agc[key]
    else:
        return agc[sel]
####################################################################################################    
def sel_ctle_map(IL='ALL', lane=None):

    lanes = get_lane_list(lane)

    for lane in lanes:
        if IL == 'SR' or IL == 'ALL' :
            ctle_map(0,1,2,lane)  
            ctle_map(1,2,3,lane)  
            ctle_map(2,3,3,lane)  
            ctle_map(3,3,4,lane)  
            ctle_map(4,3,6,lane)  
            ctle_map(5,4,5,lane)  
            ctle_map(6,5,6,lane)  
            ctle_map(7,7,7,lane)  
        elif IL == 'MR':
            wreg(0x048,0x2518,lane) # PAM4 mode, ctle_map
            wreg(0x049,0x79eb,lane) # PAM4 mode, ctle_map
            wreg(0x04a,0xbf3f,lane) # PAM4 mode, ctle_map, CTLE7=(7,7)
        elif IL == 'VSR':
            wreg(0x048,0x2518,lane) # PAM4 mode, ctle_map
            wreg(0x049,0x79eb,lane) # PAM4 mode, ctle_map
            wreg(0x04a,0xbf3f,lane) # PAM4 mode, ctle_map, CTLE7=(7,7)
        
        
        
####################################################################################################
# 
# 
####################################################################################################
def eye_check(lane=None, print_en=0):

    lanes = get_lane_list(lane)
    ths_sel_list = [x for x in range(12)]
    eye_status   = [x for x in range(0,len(lanes))] # = [x for x in range(0,len(lane_name_list))]
    num_lanes=0
    
    for ln in lanes:
        eye_status[num_lanes] = 0
        #record current setting
        bp1_org = bp1(lane=ln)[ln][0:2]
        
        #### Clear BP2, Toggle BP1 and Continue to BP1 at state 0x12
        bp2(0,0x12,lane=ln)
        bp1(0,0x12,lane=ln)
        sm_cont_01(lane=ln)
        bp1(1,0x12,lane=ln) 
        wait_for_bp1_timeout=0
        while True:
            if bp1(lane=ln)[ln][-1]: 
                ma = rreg(c.rx_cntr_sel_addr,ln) # read_cntr_sel[3:0]
                if ma==0xc:
                    break
                else:
                    bp1(0,0x12,ln)
                    sm_cont_01(lane=ln)
                    bp1(1,0x12,ln)
            else:
                wait_for_bp1_timeout+=1
                if wait_for_bp1_timeout>1000:
                    if print_en: print("\nEyeCheck: Lane %s ***>> Timed out waiting for read_state_counter=0, before starting Getting Taps"%(lane_name_list[ln]))
                    eye_status[num_lanes]+=32
                    break
                    

        ths = []
        for tap in ths_sel_list:
            wreg (c.rx_pam4_dfe_sel_addr, tap, ln)    # [0x88,  [7,4]] Ths_q_sel[3:0]
            readout  = rreg(c.rx_pam4_dfe_rd_addr, ln)
            readout2 = rreg(c.rx_pam4_dfe_rd_addr, ln)
            if readout != readout2:
                readout = rreg(c.rx_pam4_dfe_rd_addr, ln)
            result = ((float(readout)/2048.0)+1.0) % 2.0 - 1.0
            ths.append(result)
 
        bp1(0,0x12,ln)
        sm_cont_01(lane=ln)
        
        ###### Now analyze the data to get Eye Status
        case_2_margin = 0.080 #4
        case_3_margin = 0.050 #2
        case_4_margin = 0.160 #8

        
        if (ths[0]>ths[1])|(ths[1]>ths[2])|(ths[3]>ths[4])|(ths[4]>ths[5])|(ths[6]>ths[7])|(ths[7]>ths[8])|(ths[9]>ths[10])|(ths[10]>ths[11]):
            eye_status[num_lanes] += 1

        if (abs((ths[0]+ths[2])/2-ths[1])>case_2_margin)|(abs((ths[3]+ths[5])/2-ths[4])>case_2_margin)|(abs((ths[6]+ths[8])/2-ths[7])>case_2_margin)|(abs((ths[9]+ths[11])/2-ths[10])>case_2_margin):
            eye_status[num_lanes] += 2
            
        if (abs(ths[0]+ths[11])>3*case_3_margin) | (abs(ths[1]+ths[10])>3*case_3_margin) | (abs(ths[2]+ths[9])>3*case_3_margin)|(abs(ths[3]+ths[8])>3*case_3_margin)|(abs(ths[4]+ths[7])>3*case_3_margin)| (abs(ths[5]+ths[6])>3*case_3_margin) :
        #if (abs(ths[1]+ths[10])>3*case_3_margin) or (abs(ths[4]+ths[7])>3*case_3_margin) :
            eye_status[num_lanes] += 4
            
        if (abs((ths[3]-ths[0])-(ths[6]-ths[3]))> case_4_margin)|(abs((ths[4]-ths[1])-(ths[7]-ths[4]))> case_4_margin)|(abs((ths[5]-ths[2])-(ths[8]-ths[5]))> case_4_margin):
          #if(gDebugTuning):print '\nDetected Smart Reset case4!\n',
              eye_status[num_lanes] += 8

        if print_en: 
            ###### Print each ths next to its symmetrical side
            print("\nEyeCheck: Lane %s, Tap  Value Tap  Value   Diff")%(lane_name_list[ln]),
            for tap in range(0,int(len(ths)/2)):
                tap1=tap; tap2=len(ths)-1-tap
                print("\nEyeCheck: Lane %s, %3d %6.3f %3d %6.3f %6.3f"%(lane_name_list[ln],tap1,ths[tap1], tap2, ths[tap2],abs(ths[tap1]+ths[tap2]))),

            if eye_status[num_lanes]== 0: print '\nEyeCheck: Lane %s, Eye Status: Normal\n'%(lane_name_list[ln]),
            if eye_status[num_lanes] &32: print '\nEyeCheck: Lane %s, *** Eye Status: EYE CLOSED!\n'%(lane_name_list[ln]),
            if eye_status[num_lanes] & 1: print '\nEyeCheck: Lane %s, *** Eye Status: ABNORMAL case 1'%(lane_name_list[ln]),
            if eye_status[num_lanes] & 2: print '\nEyeCheck: Lane %s, *** Eye Status: ABNORMAL case 2'%(lane_name_list[ln]),
            if eye_status[num_lanes] & 4: print '\nEyeCheck: Lane %s, *** Eye Status: ABNORMAL case 3'%(lane_name_list[ln]),
            if eye_status[num_lanes] & 8: print '\nEyeCheck: Lane %s, *** Eye Status: ABNORMAL case 4'%(lane_name_list[ln]),
            print('\n'),

        num_lanes+=1 # check next lane
        
    return eye_status
####################################################################################################
# 
# 
####################################################################################################
def eye_check_nrz_orig(lane=None, print_en=1):

    lanes = get_lane_list(lane)
    ths_reg=[0]*8

    #ths_reg_list = [0x114,0x115,0x116,0x117,0x118,0x119,0x189,0x18a]
    ths_reg[0]=[0x114,[15,4]]
    ths_reg[1]=[0x114, [3,0], 0x115,[15, 8]]
    ths_reg[2]=[0x115, [7,0], 0x116,[15,12]]
    ths_reg[3]=[0x116,[11,0]]
    ths_reg[4]=[0x117,[15,4]]
    ths_reg[5]=[0x117, [3,0], 0x118,[15, 8]]
    ths_reg[6]=[0x118, [7,0], 0x119,[15,12]]
    ths_reg[7]=[0x119,[11,0]]
    
    max_eye_margin_adapt = [0x189,[15,4]]
    max_eye_tap_unbalance= [0x189, [3,0],0x18a,[15,8]]
    max_eye_delta_adapt  = [0x18a, [7,1]]
    
    ths_sel_list = [x for x in range(8)]
    eye_status   = [x for x in range(0,len(lanes))] # = [x for x in range(0,len(lane_name_list))]
    num_lanes=0
    
    for ln in lanes:
        eye_status[num_lanes] = 0
        ths = []
        for tap in ths_sel_list:
            ths.append(rreg(ths_reg[tap], ln))

        if print_en: 
            ###### Print each ths next to its symmetrical side
            print("\nEyeCheck: Lane %s, Tap  Value Tap  Value   Diff")%(lane_name_list[ln]),
            for tap in range(0,int(len(ths)/2)):
                tap1=tap; tap2=len(ths)-1-tap
                print("\nEyeCheck: Lane %s, %3d 0x%04x %3d 0x%04x %d"%(lane_name_list[ln],tap1,ths[tap1], tap2, ths[tap2],abs(ths[tap1]+ths[tap2]))),

            print('\n'),
####################################################################################################
# 
# 
####################################################################################################
def eye_check_nrz(lane=None, print_en=1):

    lanes = get_lane_list(lane)
    ths_reg=[0]*8

    #ths_reg_list = [0x114,0x115,0x116,0x117,0x118,0x119,0x189,0x18a]
    ths_reg[0]=[0x114,[15,4]]
    ths_reg[1]=[0x114, [3,0], 0x115,[15, 8]]
    ths_reg[2]=[0x115, [7,0], 0x116,[15,12]]
    ths_reg[3]=[0x116,[11,0]]
    ths_reg[4]=[0x117,[15,4]]
    ths_reg[5]=[0x117, [3,0], 0x118,[15, 8]]
    ths_reg[6]=[0x118, [7,0], 0x119,[15,12]]
    ths_reg[7]=[0x119,[11,0]]
    
    max_eye_margin_adapt = [0x189,[15,4]]
    max_eye_tap_unbalance= [0x189, [3,0],0x18a,[15,8]]
    max_eye_delta_adapt  = [0x18a, [7,1]]
    
    ths_sel_list = [x for x in range(8)]
    eye_status   = [x for x in range(0,len(lanes))] # = [x for x in range(0,len(lane_name_list))]
    num_lanes=0
    
    for ln in lanes:
        eye_status[num_lanes] = 0
        ths = []
        for tap in ths_sel_list:
            readout = rreg(ths_reg[tap], ln)
            result = ((float(readout)/2048.0)+1.0) % 2.0 - 1.0
            ths.append(result)

        if print_en: 
            ###### Print each ths next to its symmetrical side
            print("\nEyeCheck: Lane %s, Tap  Value Tap  Value   Diff")%(lane_name_list[ln]),
            for tap in range(0,int(len(ths)/2)):
                tap1=tap; tap2=len(ths)-1-tap
                print("\nEyeCheck: Lane %s, %3d 0x%04x %3d 0x%04x %d"%(lane_name_list[ln],tap1,ths[tap1], tap2, ths[tap2],abs(ths[tap1]+ths[tap2]))),

####################################################################################################
# 
# Set or get f1over3 
#
# (1) f13()                get f1over3 value of gLane
# (1) f13('all')           get f1over3 values of all lanes
# (2) f13(2)               set f1over3 value of gLane to 2
# (3) f13(10, [0,15])      set f1over3 values of lanes 0 and 15  to the same value of 10
# (4) f13(10, 'all')       set delta values of all lanes to the same value of 10
#
# returns: a list of f1over3 values, for the lane numbers passed
####################################################################################################
def f13(val=None, lane=None):

    lanes = get_lane_list(lane)

    if type(val)==str and val.upper()=='ALL': 
        lanes=range(0,len(lane_name_list))
        val=None
   
    val_list_out=[]# return values goes in here
    for lane in lanes:
        if val!=None: 
            val_to_write = (val<0) and (val+0x80) or val
            wreg(c.rx_f1over3_addr, val_to_write, lane)
            #if(print_en):print ('\nF1o3 write %d, read back %d' % (val, f13())),

        f1o3 = rreg(c.rx_f1over3_addr, lane) 
        val_list_out.append((f1o3>0x40) and (f1o3-0x80) or f1o3)
    
    return val_list_out
####################################################################################################
def f13_opt(lane = None):
    lanes = get_lane_list(lane)
    result = {}
    
    for ln in lanes:
        get_lane_mode(ln)
        line_encoding = lane_mode_list[ln].lower()
        c = Pam4Reg if line_encoding == 'pam4' else NrzReg
        if line_encoding == 'pam4':
            f0,f1,f1f0_ratio = pam4_dfe(lane=ln)[ln]
            f13_best = int(f1/(f0+f1)/3.0*63)
            f13(f13_best, lane = ln)
        else:
            f13_best = 99
        result[ln] = f13_best
    return result
 
####################################################################################################
# 
#
# f1over3 table-based search
# 
####################################################################################################
def f13_table(lane=None, print_en=0):

    lanes = get_lane_list(lane)
    Slice=gSlice
    
    loops_each_dir=5  
    wait_for_eye=1.0
    init_f13_list= []
    final_f13_list=[] # contains best f13 value for each lane 
    lane_cntr=0
    for ln in lanes:
        init_f13_list.append(f13(lane=ln)[0])
        curr_delta = delta_ph(lane=ln)[0]
        if   curr_delta < -14: final_f13_val = 8 # -15 and less
        elif curr_delta < -11: final_f13_val = 7 # -14,-13,-12,-11
        elif curr_delta < -5 : final_f13_val = 6 # -10,-9,-8,-7,-6
        elif curr_delta < -1 : final_f13_val = 4 # -5,-4,-3,-2
        elif curr_delta <  3 : final_f13_val = 3 # -1,0,1,2
        elif curr_delta <  8 : final_f13_val = 2 # 3,4,5,6,7
        elif curr_delta < 13 : final_f13_val = 1 # 8,9,10,11,12
        elif curr_delta < 15 : final_f13_val = 0 # 13,14
        else: final_f13_val = -1                  # 15 and higher
        
        final_f13_list.append(final_f13_val)

        f13(final_f13_list[-1],lane=ln)
        #lane_reset_fast(lane=ln)

        # end of loop for one lane
        print ("\nSlice %d Lane %s (%s) F13 Table adjustment: Delta: %d, Orig F13: %d, New F13: %d"%(Slice, lane_name_list[ln],gEncodingMode[gSlice][ln][0].upper(),curr_delta,init_f13_list[lane_cntr],final_f13_list[lane_cntr])),
        lane_cntr+=1
        
    return init_f13_list, final_f13_list
####################################################################################################
# 
#
# f1 over 3 search rotuine
# 
# First, Start f1o3 sweep from CENTER and to LEFT: until eye_status==0 goes away
# Next, sweep f1o3 from CENTER+1 to RIGHT: until eye_status=0 goes away
# Last, picks the middle of all good f13 values
#
# can be repeated for multiple lanes 
#
####################################################################################################
def f13_search(init_f13_val=None, lane=None, print_en=0):
    lanes = get_lane_list(lane)

    loops_each_dir=5  
    wait_for_eye=1.0
    optimum_f13_list=[] # contains best f13 value for each lane    
    good_f13_list=[]    # contains list of good f13 values per lane
    lane_cntr=0
    for lane in lanes:
    
        good_f13_list.append([])
        if init_f13_val!=None:
            f13(init_f13_val,lane) # if value passed to start search with, write it to f13
        else:
            curr_delta = delta_ph(lane=lane)[0]
            if   curr_delta < -7: init_f13_val = 7
            elif curr_delta < -5: init_f13_val = 4
            elif curr_delta <  0: init_f13_val = 1
            elif curr_delta <  5: init_f13_val = 0
            elif curr_delta < 10: init_f13_val = -1
            else: init_f13_val = -2
            
            f13(val=init_f13_val,lane=lane)
            
        cal=rreg(0x087,lane)     # store super-cal disable 
        wreg(0x087,0x0800,lane)  # super-cal disabled 
    
        center_f13_val = f13(lane=lane)[0]  # search will start with this center value 
        dir=0 # first direction
        total_f13_tested_one_dir=0
        found_one_good_f13=0
        test_f13_val=center_f13_val # start first dirction with the center value
        step_size=1
        
        while (total_f13_tested_one_dir<=loops_each_dir):
            total_f13_tested_one_dir+=1 # increment num of f13 tested
            f13(test_f13_val,lane) # write f13 test value to register
            lane_reset_fast(lane=lane) # lane reset and get eye status
            time.sleep(wait_for_eye)
            eye_status = eye_check(lane=lane)[0]
            if(print_en):print ('\nF13 Search: Lane %s, Dir=%d, Loop %2d, F1o3: %3d, EyeStatus %d ' % (lane_name_list[lane],dir,total_f13_tested_one_dir, f13(lane=lane)[0], eye_status)),
            if (eye_status==0): # good eye status? add this f13 value to the good_f13_list
                good_f13_list[lane_cntr].append(test_f13_val)
                found_one_good_f13=1
            elif (eye_status!=0 and found_one_good_f13==1): # if ran into a bad one after one or more good ones, stop this directionand next one if 
                total_f13_tested_one_dir=loops_each_dir
            if dir==0 and total_f13_tested_one_dir==loops_each_dir: # done with first direction
                dir=1
                total_f13_tested_one_dir=0
                found_one_good_f13=0
                test_f13_val=center_f13_val
                step_size=-1
            elif dir==1 and total_f13_tested_one_dir==loops_each_dir: # done with both direction
                break
            test_f13_val+= step_size
            
        good_f13_list[lane_cntr].sort() # ensure good_f13_list is in sequential order
        if len(good_f13_list[lane_cntr])>0: 
            optimum_f13_list.append(good_f13_list[lane_cntr][int(len(good_f13_list[lane_cntr])/2)]) # get middle value
            if(print_en):print ('\nF13 Search: Lane %s, Best F1over3 = %d'%(lane_name_list[lane],optimum_f13_list[-1]))
        else: 
            optimum_f13_list.append(center_f13_val)
            if(print_en):print ('\nF13 Search: Lane %s, Could not find a good F1over3. Kept Original value of %d'%(lane_name_list[lane],optimum_f13_list[-1]))

        f13(optimum_f13_list[-1],lane)
        lane_reset_fast(lane=lane)
        wreg(0x087,cal,lane) # restore super-cal setting
        lane_cntr+=1
        # end of loop for one lane
        
    return optimum_f13_list, good_f13_list

####################################################################################################
# 
# 
####################################################################################################
def f13_search_orig(init_f13_val=None, lane=gLane, print_en=0):
    lanes = get_lane_list(lane)

    print_en=0
    loops_each_dir=5
    wait=.01
    for lane in lanes:
        if init_f13_val!=None:
            f13(init_f13_val,lane) # if value passed to start search with, write it to f13
            
        init_val = f13(lane=lane)[0]    
        found_optimum_f13_list = False
        
        # incrementing f13 until found optimum setting
        i=0
        while (i<loops_each_dir):
            #eye_status = smart_reset()
            lane_reset_fast(lane=lane)
            time.sleep(wait)
            eye_status = eye_check(lane=lane)
            if(print_en):print ('\nF13 Search: Lane %s, Dir=Up,  Loop %d, F1o3: %d, EyeStatus %d ' % (lane_name_list[lane],i, f13()[0], eye_status[0])),
            if (eye_status!=0):
                i+=1
                cur_f13=f13(lane=lane)[0]
                if i<loops_each_dir: f13(cur_f13+1)
            else: 
                found_optimum_f13_list = True
                break
                
        # if above not successful, try decrementing f13         
        if found_optimum_f13_list == False:
            f13(init_val,lane) # first, go back to the starting value
            i=0
            while (i<loops_each_dir):
                #eye_status = smart_reset()
                lane_reset_fast(lane=lane)
                time.sleep(wait)
                eye_status = eye_check(lane=lane)
                if(print_en):print ('\nF13 Search: Lane %s, Dir=Dwn, Loop %d, F1o3: %d, EyeStatus %d ' % (lane_name_list[lane],i, f13()[0], eye_status[0])),
                if (eye_status!=0):
                    i+=1
                    cur_f13=f13(lane=lane)[0]
                    if i<loops_each_dir: f13(cur_f13-1)
                else: 
                    found_optimum_f13_list = True
                    break
                    
        # if still no good case, go back to the starting value            
        if found_optimum_f13_list == False:
            f13(init_val,lane)
            lane_reset_fast(lane=lane)
        
    return found_optimum_f13_list
    
####################################################################################################
# 
# State Machine Continue, reversed toggle direction from main python file, here to toggle 0->1
####################################################################################################
def sm_cont_01(lane = None):
    if lane==None: lane=gLane
    wreg(c.rx_sm_cont_addr, 0, lane)
    time.sleep(0.001)
    wreg(c.rx_sm_cont_addr, 1, lane)
    #time.sleep(0.001)
    #wreg(c.rx_sm_cont_addr, 0, lane)

####################################################################################################
# 
# GET DFE Taps when FW not loaded
####################################################################################################
def sw_pam4_dfe(lane=None, print_en=0):
    global gPam4_En; gPam4_En=1
    lanes = get_lane_list(lane)
    result = {}
    for ln in lanes:
        get_lane_mode(ln)
        if lane_mode_list[ln] != 'pam4':
            result[ln] = -1,-1,-1
        else: # PAM4
            ths_list = []
            for val in range(12):
                wreg(c.rx_pam4_dfe_sel_addr, val, ln)
                time.sleep(0.01)
                readout = rreg(c.rx_pam4_dfe_rd_addr, ln)
                readout2 = rreg(c.rx_pam4_dfe_rd_addr, ln)
                if readout == readout2:
                    this_ths = ((float(readout)/2048.0)+1.0) % 2.0 - 1.0
                    if print_en: print(" %2d 0x%04X %6.3f"%(val,readout,this_ths))
                else:
                    readout = rreg(c.rx_pam4_dfe_rd_addr, ln)
                    this_ths = ((float(readout)/2048.0)+1.0) % 2.0 - 1.0
                    if print_en: print("*%2d 0x%04X %6.3f"%(val,readout,this_ths))
                ths_list.append(this_ths)
              
            f0 = (-3/16)*((ths_list[0]-ths_list[2])+(ths_list[3]-ths_list[5])+(ths_list[6]-ths_list[8])+(ths_list[9]-ths_list[11]))
            f1 = (-3/20)*((ths_list[0]+ths_list[1]+ths_list[2]-ths_list[9]-ths_list[10]-ths_list[11])+(1/3)*(ths_list[3]+ths_list[4]+ths_list[5]-ths_list[6]-ths_list[7]-ths_list[8]))
            f1_f0 = 0 if f0==0 else f1/f0
            result[ln] = f0, f1, f1_f0
        
    return result   
####################################################################################################
# 
# GET DFE Taps by the FW
####################################################################################################
def fw_pam4_dfe(lane=None,print_en=0):
    lanes = get_lane_list(lane)
    result = {}
    for ln in lanes:
        get_lane_mode(ln)
        if lane_mode_list[ln] != 'pam4':
            result[ln] = -1,-1,-1
        else: # PAM4
            readout = BE_info_signed(ln, 10, 1, 12)
            ths_list = []      
            for val in range(12):
                ths_list.append(((float(readout[val])/2048.0)+1.0) % 2.0 - 1.0)
                if print_en: print("%2d 0x%04X %6.3f"%(val,readout,result))
            
            f0 = (-3/16)*((ths_list[0]-ths_list[2])+(ths_list[3]-ths_list[5])+(ths_list[6]-ths_list[8])+(ths_list[9]-ths_list[11]))
            f1 = (-3/20)*((ths_list[0]+ths_list[1]+ths_list[2]-ths_list[9]-ths_list[10]-ths_list[11])+(1/3)*(ths_list[3]+ths_list[4]+ths_list[5]-ths_list[6]-ths_list[7]-ths_list[8]))
            f1_f0 = 0 if f0==0 else f1/f0
            result[ln] = f0, f1, f1_f0
        
    return result
####################################################################################################
# 
# GET DFE Taps by the FW if loaded, or by software
####################################################################################################
def pam4_dfe(lane=None,print_en=0):
    result = {}
    if fw_loaded:
        result = fw_pam4_dfe(lane,print_en)
    else:
        result = sw_pam4_dfe(lane,print_en)
    return result
####################################################################################################
# 
# GET DFE Taps, NRZ or PAM4
####################################################################################################
def dfe(lane=None,print_en=0):
    lanes = get_lane_list(lane)
    result = {}
    for ln in lanes:
        get_lane_mode(ln)
        if lane_mode_list[ln] == 'pam4':
            result[ln] = pam4_dfe(ln,print_en)
        else: # NRZ
            result[ln] = nrz_dfe(ln)
        
    return result
####################################################################################################
# 
# Fast Tap Update. used before calling delta search loop
####################################################################################################
def fast_tap_update ( tap=1, enable=1, lane=None):
    if lane==None: lane=gLane
    print_en=0
    
    #record current setting
    org1 = rreg(c.rx_margin_patt_dis_owen_addr, lane)
    org2 = rreg(c.rx_margin_patt_dis_ow_addr  , lane)
    org3 = rreg(c.rx_mu_ow_addr               , lane)
    org4 = rreg(c.rx_iter_s6_addr             , lane)
    org5 = rreg(c.rx_timer_meas_s6_addr       , lane)
    org6 = rreg(c.rx_bp1_en_addr              , lane)
    org7 = rreg(c.rx_bp1_st_addr              , lane)
    org8 = rreg(c.rx_bp2_en_addr              , lane)
    org9 = rreg(c.rx_bp2_st_addr              , lane)
    org10= rreg(c.rx_sm_cont_addr             , lane)
    
    # program registers for , b0, dir, pattern_p and pattern_n
    b0=1; dir=0
    wreg(c.rx_fixed_patt_mode_addr, tap, lane) # select tap 1
    wreg(c.rx_fixed_patt_b0_addr, b0, lane)
    wreg(c.rx_fixed_patt_dir_addr, dir, lane)
    
    #### Clear BP2, Toggle BP1 and Continue to BP1 at state 0x12
    bp2(0,0x12,lane=lane)
    bp1(0,0x12,lane=lane)
    sm_cont_01(lane=lane)
    bp1(1,0x12,lane=lane)    

    wait_for_bp1_timeout = 0
    while True:
        if bp1(lane=lane)[lane][-1]: 
            ma = rreg(c.rx_mj_addr,lane) # wait for the lowest 2 bits to become '00'
            if ma == 0:
                break
            else:
                bp1(0,0x12,lane)
                sm_cont_01(lane=lane)
                bp1(1,0x12,lane)
        else:
            wait_for_bp1_timeout += 1
            if wait_for_bp1_timeout > 500:
                print("\nFast Tap Update ***>> Timed out waiting for read_state_counter=0, before starting Getting Taps")
                break
    if enable:    
        wreg(c.rx_margin_patt_dis_owen_addr, 1, lane)
        wreg(c.rx_margin_patt_dis_ow_addr, 0, lane)
        wreg(c.rx_iter_s6_addr, 1, lane)
        wreg(c.rx_timer_meas_s6_addr, 8, lane)
    else: # back to normal operation settings
        wreg(c.rx_margin_patt_dis_owen_addr, 0, lane)
        wreg(c.rx_margin_patt_dis_ow_addr, 0, lane)
        wreg(c.rx_iter_s6_addr, 6, lane)            # 0x09[3:0] iteration_count_S6[3:0]
        wreg(c.rx_timer_meas_s6_addr, 8, lane)
               
    #wreg(c.rx_margin_patt_dis_owen_addr, org1 ,lane)
    #wreg(c.rx_margin_patt_dis_ow_addr  , org2 ,lane)
    wreg(c.rx_mu_ow_addr               , org3 ,lane)
    #wreg(c.rx_iter_s6_addr             , org4 ,lane)
    #wreg(c.rx_timer_meas_s6_addr       , org5 ,lane)
    wreg(c.rx_bp1_en_addr              , org6 ,lane)
    wreg(c.rx_bp1_st_addr              , org7 ,lane)
    wreg(c.rx_bp2_en_addr              , org8 ,lane)
    wreg(c.rx_bp2_st_addr              , org9 ,lane)
    #wreg(c.rx_sm_cont_addr             , org10,lane)
    sm_cont_01(lane=lane)
####################################################################################################
# 
# Set or get DELTA Phase
#
# (1) delta_ph()                get delta value of gLane
# (1) delta_ph('all')           get delta values of all lanes
# (2) delta_ph(2)               set delta value of gLane to 2
# (3) delta_ph(10, [0,15])      set delta values of lanes 0 and 15  to the same value of 10
# (4) delta_ph(10, 'all')       set delta values of all lanes to the same value of 10
#
# returns: a list of delta values, for the lane numbers passed
####################################################################################################
def delta_ph(val=None, lane=None):

    lanes = get_lane_list(lane)

    if type(val)==str and val.upper()=='ALL': 
        lanes=range(0,len(lane_name_list))
        val=None

    val_list_out=[]# return values goes in here
    for ln in lanes:
        get_lane_mode(ln)
        this_lane_mode = gEncodingMode[gSlice][ln][0]
        c = NrzReg if this_lane_mode == 'nrz' else Pam4Reg
        ######### PAM4 Mode
        if this_lane_mode.upper()=='PAM4':
            if val!=None:
                delta_to_write= (val<0) and (val+0x80) or val
                wreg(c.rx_delta_owen_addr, 1, ln)
                wreg(c.rx_delta_ow_addr, delta_to_write, ln)
                
            delta_val= rreg(c.rx_delta_ow_addr,  ln)
            #val_list_out.append((delta_val>0x40) and (delta_val-0x80) or delta_val)

        ######### NRZ Mode
        else: 
            if val!=None:
                wreg(dfe_delta_addr, val, ln)                
            else:
                delta_val = rreg(c.rx_delta_addr,  ln)
                
        val_list_out.append((delta_val>0x40) and (delta_val-0x80) or delta_val)
        
    return val_list_out
        
####################################################################################################
# 
# Get Fml Value
####################################################################################################
def get_fm1(lane=None):
    
    lanes = get_lane_list(lane)
    
    fm1_val =[]
    for lane in lanes:
        # f-1 quick update before starting delta search
        fast_tap_update(1,1,lane) # f-1 Update enabled
        time.sleep(0.1)
        fast_tap_update(1,0,lane) # back to normal operation settings
        
        fm1_val.append(((rreg(c.rx_pam4_tap_addr,lane))+64)%128 - 64) # read_fm1_q[6:0]
    
    return fm1_val
####################################################################################################
#
# N E W   Delta Search, based on f-1 tap only
####################################################################################################
def delta_search(lane=None, print_en=0):
    
    lanes = get_lane_list(lane)

    Slice=gSlice
    target_fm1=40
    last_retry=1
    final_delta=[]
    loops=20
    
    for ln in lanes:
        orig_delta = delta_ph(lane=ln)[0]
        for loop in range(loops):
            curr_delta = delta_ph(lane=ln)[0]
            new_delta = curr_delta
            
            #fm1_val = get_fm1(lane=ln)[0]
            iter = 5
            fm1_val = 0
            for i in range(iter):
                fm1_val_list = sw_pam4_isi (b0=1, dir=0, isi_tap_range=[0,1], lane=ln) #[0]
                fm1_val += fm1_val_list[1]/5
            if(print_en):print("\nDeltaSearch: Lane %s, Loop %2d , Delta= %-+3d , Fm2= %-+4d , Fm1= %-+4d"%(lane_name_list[ln],loop,curr_delta, fm1_val_list[0],fm1_val_list[1])),
            delta_jump=0
            if abs(fm1_val) < 1000:
                if (fm1_val-target_fm1 < -100):   # very negative difference
                    delta_jump = +2 
                elif (fm1_val-target_fm1 > 100):  # very positive difference
                    delta_jump = -2
                elif (fm1_val-target_fm1 < -60):  #  mid negative difference
                    delta_jump = +1
                elif (fm1_val-target_fm1 > 60):   #  mid positive difference
                    delta_jump = -1
                # elif (fm1_val-target_fm1 < -25): #  small negative difference
                    # delta_jump = +1
                # elif (fm1_val-target_fm1 > 25):  #  small positive difference
                    # delta_jump = -1 
                else: # difference is less than 30
                    delta_jump=0 
                    if last_retry > 0: # if very small difference, give it one more try before exiting loop
                        last_retry -= 1

            if(print_en):print('TgtFm1= %-+3d , Delta jump by %-+2d'%(target_fm1,delta_jump)),
               
            new_delta = curr_delta+delta_jump
            if last_retry == 0 or new_delta>=16 or new_delta<=-16:
                if new_delta<=-16:
                    print("\n*** Delta Search: Delta Min Limit Hit!  More TX Pre-Emphasis is Recommended!\n\n")
                if new_delta>=16:
                    print("\n*** Delta Search: Delta MAX Limit Hit!  Less TX Pre-Emphasis is Recommended!\n\n")
                break

            #if (new_delta<-0x20): new_delta=-0x20
            #elif (new_delta>0x20): new_delta=0x20
            delta_ph(new_delta, lane=ln)
            #if(print_en):print ('-> New Delta: %d' % (new_delta)),
            
            # stop if ISI change is too small (twice) or delta is larger than +/-15
                
        final_delta.append(new_delta) # store final delta for this lane
        if(print_en):print ('\n')
        print ("\nSlice %d Lane %s (%s) Delta Search: OrigDelta: %3d, NewDelta: %3d"%(Slice, lane_name_list[ln],gEncodingMode[gSlice][ln][0].upper(),orig_delta,new_delta)),
        
    return final_delta
####################################################################################################
# 
# Delta Search loop
####################################################################################################
def delta_search_loop_orig(loop=5, force_clean=1, lane=None):

    if lane==None: lane=gLane   
    if(print_en):print ("\n\nDelta Search Loop: START"),

    
    i = 0
    while i<loop:
        i = i+1
        #time.sleep(0.1)
        delta_search(force_clean, lane)
      
        #lr(lane=lane)
    
    if(print_en):print ('\nDelta Search Loop: END')

####################################################################################################
# 
# Delta Search
####################################################################################################
def delta_search_orig(force_clean=1, lane=None):
    
    if lane==None: lane=gLane
    print_en=0
    target_fm1=0
    
    # f-1 quick update before starting delta search
    fast_tap_update(1,1,lane) # f-1 Update enabled
    time.sleep(0.1)
    fast_tap_update(1,0,lane) # back to normal operation settings
    
    #record every register that is being touched
    org1 = rreg(c.rx_margin_patt_dis_owen_addr, lane)
    org2 = rreg(c.rx_margin_patt_dis_ow_addr  , lane)
    org3 = rreg(c.rx_mu_ow_addr               , lane)
    org4 = rreg(c.rx_iter_s6_addr             , lane)
    org5 = rreg(c.rx_timer_meas_s6_addr       , lane)
    org6 = rreg(c.rx_bp1_en_addr              , lane)
    org7 = rreg(c.rx_bp1_st_addr              , lane)
    org8 = rreg(c.rx_bp2_en_addr              , lane)
    org9 = rreg(c.rx_bp2_st_addr              , lane)
    org10= rreg(c.rx_sm_cont_addr             , lane)
    org11= rreg(c.rx_delay_loop_freeze_addr   , lane)
    org12= rreg(c.rx_theta_update_mode_addr   , lane)
    org13= rreg(c.rx_delta_owen_addr          , lane)
    org14= rreg(c.rx_fixed_patt_mode_addr     , lane)
    org15= rreg(c.rx_fixed_patt_b0_addr       , lane)
    org16= rreg(c.rx_fixed_patt_dir_addr      , lane)
    org17= rreg(c.rx_pam4_dfe_sel_addr        , lane)
    bp1_org = bp1(lane=lane)[lane][0:2]
    
    # updn mode
    wreg(c.rx_delay_loop_freeze_addr ,1,lane) # freeze delay loop
    wreg(c.rx_theta_update_mode_addr, 7,lane) # theta2,3,4 update mode is data, not clock comp mode
    wreg(c.rx_delay_loop_freeze_addr ,0,lane) # unfreeze delay loop
    wreg(c.rx_delta_owen_addr, 1,lane)  # [0x12, [13]]    

    # program registers for , b0, dir, pattern_p and pattern_n
    tap=1; b0=1; dir=0
    wreg(c.rx_fixed_patt_mode_addr, tap, lane) # select tap 1
    wreg(c.rx_fixed_patt_b0_addr, b0, lane)
    wreg(c.rx_fixed_patt_dir_addr, dir, lane)
    time.sleep(0.3)

    #### Clear BP2, Toggle BP1 and Continue to BP1 at state 0x12
    bp2(0,0x12,lane=lane)
    bp1(0,0x12,lane=lane)
    sm_cont_01(lane=lane)
    bp1(1,0x12,lane=lane)    
    
    wait_for_bp1_timeout=0
    while True:
        if bp1(lane=lane)[lane][-1]: 
            ma = rreg(c.rx_cntr_sel_addr,lane) # read_cntr_sel[3:0]
            if ma==0xc:
                break
            else:
                bp1(0,0x12,lane)
                sm_cont_01(lane=lane)
                bp1(1,0x12,lane)
        else:
            wait_for_bp1_timeout+=1
            if wait_for_bp1_timeout>1000:
                print("\nDelta Search ***>> Timed out waiting for read_state_counter=0, before starting Getting Taps")
                break

    ths_sel_list = [x for x in range(12)]
    ths_list = []
    for tap in ths_sel_list:
        wreg (c.rx_pam4_dfe_sel_addr, tap, lane)    # [0x88,  [7,4]] Ths_q_sel[3:0]
        readout  = rreg(c.rx_pam4_dfe_rd_addr, lane)
        readout2 = rreg(c.rx_pam4_dfe_rd_addr, lane)
        if readout != readout2:
            readout = rreg(c.rx_pam4_dfe_rd_addr, lane)
        result = ((float(readout)/2048.0)+1.0) % 2.0 - 1.0
        if print_en: print("\n%2d 0x%04X %6.3f"%(tap,readout,result)),
        ths_list.append(result)
        ths_list.append(result)
        
    f0 = int((-3)*((ths_list[0]-ths_list[2])+(ths_list[3]-ths_list[5])+(ths_list[6]-ths_list[8])+(ths_list[9]-ths_list[11]))*4)
    f1 = int(((-48)*(ths_list[0]+ths_list[1]+ths_list[2]-ths_list[9]-ths_list[10]-ths_list[11])+(-16)*(ths_list[3]+ths_list[4]+ths_list[5]-ths_list[6]-ths_list[7]-ths_list[8]))/5)
    fm1 = (((rreg(c.rx_pam4_tap_addr,lane))+64)%128 - 64) # read_fm1_q[6:0]

    curr_delta = delta_ph(lane=lane)
    new_delta = curr_delta
    
    if(print_en):print("\nDelta Search: Delta=%2d, F0= %2d F1= %2d Fm1= %2d" % (curr_delta, f0, f1, fm1)),
    #target_fm1 = 1
    
    if (fm1<target_fm1):
        fm1s = (fm1<target_fm1-3) and -3 or fm1
        if (f1>-2): 
            new_delta = curr_delta-fm1s
            if(print_en):print ' Fm1<0, F1>-2 => delta-fm1s(%d)'%fm1s,
    
        else: 
            if(print_en):print ' Fm1<0, F1<-1, TX pre/post overboost, delta+1',
            if (fm1<target_fm1-2): 
                new_delta = curr_delta+1
    elif (fm1>target_fm1):
        fm1s = (fm1>target_fm1+3) and 3 or fm1
        if (f0>4*f1): 
            if(print_en):print ' Fm1>0, F0/F1 too large => Force Clean=1, delta-fm1s(%d)'%fm1s,
            new_delta = curr_delta-fm1s
        elif force_clean: 
            if(print_en):print ' Fm1>0, ForceClean=1, delta-1',
            new_delta = curr_delta-1
        else: 
            if(print_en):print ' Fm1>0, ForceClean=0, need more TX pre, delta-1',
            if (fm1>target_fm1+2): 
                new_delta = curr_delta-1
    elif force_clean:
        if(print_en):print' Fm1=0, ForceClean=1, delta not changed',
        new_delta = curr_delta
    else:
        if (f0<3*f1) and (f1>0): 
            if(print_en):print' Fm1=0, f0<3f1, ForceClean=0, delta+1',
            new_delta = curr_delta+1
        else:
            if(print_en):print' Fm1=0, ForceClean=0, delta not changed',


    if (new_delta<-0x20): new_delta=-0x20
    elif (new_delta>0x20): new_delta=0x20

    delta_ph(new_delta, lane=lane)
    if(print_en):print ('-> New Delta: %d' % (new_delta)),
    
    #restore every register that was touched
    wreg(c.rx_margin_patt_dis_owen_addr, org1 ,lane)
    wreg(c.rx_margin_patt_dis_ow_addr  , org2 ,lane)
    wreg(c.rx_mu_ow_addr               , org3 ,lane)
    wreg(c.rx_iter_s6_addr             , org4 ,lane)
    wreg(c.rx_timer_meas_s6_addr       , org5 ,lane)
    wreg(c.rx_bp1_en_addr              , org6 ,lane)
    wreg(c.rx_bp1_st_addr              , org7 ,lane)
    wreg(c.rx_bp2_en_addr              , org8 ,lane)
    wreg(c.rx_bp2_st_addr              , org9 ,lane)
    #wreg(c.rx_sm_cont_addr             , org10,lane)
    wreg(c.rx_delay_loop_freeze_addr   , org11,lane)
    wreg(c.rx_theta_update_mode_addr   , org12,lane)
    wreg(c.rx_delta_owen_addr          , org13,lane)
    wreg(c.rx_fixed_patt_mode_addr     , org14,lane)
    wreg(c.rx_fixed_patt_b0_addr       , org15,lane)
    wreg(c.rx_fixed_patt_dir_addr      , org16,lane)
    wreg(c.rx_pam4_dfe_sel_addr        , org17,lane)
    sm_cont_01(lane=lane)
####################################################################################################
# 
# Set or get FFE Polarity Settings
#
# (1) ffe_pol()                get FFE Polarity values of gLane
# (1) ffe_pol('all')           get FFE Polarity values of all lanes
# (2) ffe_pol(1,0,1,0)         set FFE Polarity values of gLane to 1010
# (3) ffe_pol(1,0,1,0, [0,15]) set FFE Polarity values of lanes 0 and 15  to the same values of 1010
# (4) ffe_pol(1,0,1,0, 'all')  set FFE Polarity values of all lanes to the same values of 1010
#
# returns: a list of FFE Polarity, for the lane numbers passed
####################################################################################################
def ffe_pol(ffe_pol1=None, ffe_pol2=None, ffe_pol3=None, ffe_pol4=None, lane=None):

    lanes = get_lane_list(lane)

    if type(ffe_pol1)==str and ffe_pol1.upper()=='ALL': # treat this case as read all lanes' taps
        lanes=range(0,len(lane_name_list))
        ffe_pol1=None

    ffe_pol_per_lane=[]   # contains list of ffe tap polarity settings per lane
    lane_cntr=0
    for lane in lanes:
        ffe_pol_per_lane.append([])
        if ffe_pol1!=None: #and ffe_pol2!=None and ffe_pol3!=None and ffe_pol4!=None:
            wreg(c.rx_ffe_pol1_addr,ffe_pol1,lane)
        if ffe_pol2 != None:
            wreg(c.rx_ffe_pol2_addr,ffe_pol2,lane)
        if ffe_pol3 != None:
            wreg(c.rx_ffe_pol3_addr,ffe_pol3,lane)
        if ffe_pol4 != None:
            wreg(c.rx_ffe_pol4_addr,ffe_pol4,lane)
         
        ffe_pol_per_lane[lane_cntr].append(rreg(c.rx_ffe_pol1_addr,lane))
        ffe_pol_per_lane[lane_cntr].append(rreg(c.rx_ffe_pol2_addr,lane))
        ffe_pol_per_lane[lane_cntr].append(rreg(c.rx_ffe_pol3_addr,lane))
        ffe_pol_per_lane[lane_cntr].append(rreg(c.rx_ffe_pol4_addr,lane))
        
        lane_cntr+=1
        
    return ffe_pol_per_lane
####################################################################################################
# 
# 
####################################################################################################
def ffe_nbias(nbias=None,lane=None):
    #rx_ffe_nbias_main_addr = [0x1d6,[15,12]] ############ added into 'phoenix_reg.py'
    #rx_ffe_nbias_sum_addr = [0x1d6,[11,8]]   ############ added into 'phoenix_reg.py'
    
    lanes = get_lane_list(lane)

    ffe_nbias_per_lane=[]   # contains list of ffe tap polarity settings per lane
    lane_cntr=0
    for lane in lanes:
        if nbias!=None:
            wreg(c.rx_ffe_nbias_main_addr,Bin_Gray(nbias),lane)
        else:
            ffe_nbias_per_lane.append(Gray_Bin(rreg(c.rx_ffe_nbias_main_addr,lane)))
        lane_cntr+=1
 
    return ffe_nbias_per_lane

####################################################################################################
# 
# FFE POLARITY SEARCH
####################################################################################################
def ffe_pol_search(lane=None, print_en=0):
    
    lanes = get_lane_list(lane)

    for lane in lanes:
        ffe_k1_pol = rreg(c.rx_ffe_pol1_addr,lane)
        ffe_k2_pol = rreg(c.rx_ffe_pol2_addr,lane)
        ffe_k3_pol = rreg(c.rx_ffe_pol3_addr,lane)
        ffe_k4_pol = rreg(c.rx_ffe_pol4_addr,lane)

        if(print_en):print("\nFFE Taps Pol, Lane %s, Before:(%2d,%2d,%2d,%2d)"%(lane_name_list[lane],ffe_k1_pol,ffe_k2_pol,ffe_k3_pol,ffe_k4_pol)),
        [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin] = ffe_taps(lane=lane)[lane]
        if(print_en):print("\nFFE Taps Val, Lane %s, Before:(%02X,%02X,%02X,%02X,%02X,%02X)" %(lane_name_list[lane],ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin)),

        start_time = time.time()
        tap_range = range(2,6)
        tap_value = [0]*len(tap_range)
        iter = 5  
        if(print_en):print('')
        if(print_en):print('\nFFE Polarity Search: Lane %s, using ISI Taps %d to %d'%(lane_name_list[lane],tap_range[0],tap_range[-1])),
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, tap_range, lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
            #if(print_en):print('\nFFE Polarity Search:'),
            #if(print_en):print(["%6.1f"%x for x in tap_list]),
        if(print_en):print('\nFFE Polarity Search:  Lane %s')%(lane_name_list[lane]),
        if(print_en):print(["%6.1f"%x for x in tap_value])
        
        ffe_k1_step = 20
        ffe_k2_step = 10
        ffe_k3_step = 5
        ffe_k4_step = 5

        ffe_taps(s2=0x44,s1=0x8,lane=lane)
        ffe_k1_k2_step = 0
        if ffe_k4_pol==0:
            if tap_value[3]>40: 
                wreg(c.rx_ffe_pol4_addr,1,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap4 pol:1'%(lane_name_list[lane])
        else:
            if tap_value[3]<-40: 
                wreg(c.rx_ffe_pol4_addr,0,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap4 pol:0'%(lane_name_list[lane])
        
        if ffe_k3_pol==0:
            if tap_value[2]>40: 
                wreg(c.rx_ffe_pol3_addr,1,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap3 pol:1'%(lane_name_list[lane])
        else:
            if tap_value[2]<-40: 
                wreg(rx_ffe_pol3_addr,0,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap3 pol:0'%(lane_name_list[lane])
        
        if ffe_k2_pol==0:
            if tap_value[1]>60: 
                wreg(c.rx_ffe_pol2_addr,1,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap2 pol:1'%(lane_name_list[lane])
                ffe_k1_k2_step = -30
        else:
            if tap_value[1]<-60: 
                wreg(c.rx_ffe_pol2_addr,0,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap2 pol:0'%(lane_name_list[lane])
                ffe_k1_k2_step = 30
        
        tap_value[0] = tap_value[0] + ffe_k1_k2_step
        ffe_k1_main1 = Gray_Bin(rreg(c.rx_ffe_k1_msb_addr, lane))
        ffe_k1_main0 = Gray_Bin(rreg(c.rx_ffe_k1_lsb_addr, lane))
        step_old = (ffe_k1_main1 << 4) + ffe_k1_main0
        if ffe_k1_pol==0:
            if tap_value[0]>81: # was 120: 2017-05-30
                wreg(c.rx_ffe_pol1_addr,1,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 pol:1'%(lane_name_list[lane])
            elif tap_value[0]<-ffe_k1_step:
                step = int(abs(tap_value[0])/(ffe_k1_step*2))
                ffe_taps(k1 = step_old+step*17,lane=lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 Pol:%d Step:%02X' % (lane_name_list[lane],ffe_k1_pol,step_old+step*17)
        else:
            if tap_value[0]<-81: # was -120: 017-05-30
                wreg(c.rx_ffe_pol1_addr,0,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 pol:0'% (lane_name_list[lane])
            elif tap_value[0]>ffe_k1_step:
                step = int(abs(tap_value[0])/(ffe_k1_step*2))
                ffe_taps(k1 = step_old+step*17,lane=lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 Pol:%d Step:%02X' % (lane_name_list[lane],ffe_k1_pol,step_old+step*17)
        ############################################################################################

        if ffe_k2_pol==0:
            if tap_value[0]>81: # was 120: 2017-05-30
                wreg(c.rx_ffe_pol1_addr,1,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 pol:1'% (lane_name_list[lane])
            elif tap_value[0]<-ffe_k1_step:
                step = int(abs(tap_value[0])/(ffe_k2_step*2))
                ffe_taps(k2 = step_old+step*17,lane=lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 Pol:%d Step:%02X' % (lane_name_list[lane],ffe_k1_pol,step_old+step*17)
        else:
            if tap_value[0]<-81: # was -120: 017-05-30
                wreg(c.rx_ffe_pol1_addr,0,lane)
                if(print_en):print 'FFE Polarity Search: tap1 pol:0'
            elif tap_value[0]>ffe_k1_step:
                step = int(abs(tap_value[0])/(ffe_k2_step*2))
                ffe_taps(k2 = step_old+step*17,lane=lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 Pol:%d Step:%02X' % (lane_name_list[lane],ffe_k1_pol,step_old+step*17)


        if ffe_k3_pol==0:
            if tap_value[0]>81: # was 120: 2017-05-30
                wreg(c.rx_ffe_pol1_addr,1,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 pol:1'
            elif tap_value[0]<-ffe_k1_step:
                step = int(abs(tap_value[0])/(ffe_k3_step*2))
                ffe_taps(k3 = step_old+step*17,lane=lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 Pol:%d Step:%02X' % (lane_name_list[lane],ffe_k1_pol,step_old+step*17)
        else:
            if tap_value[0]<-81: # was -120: 017-05-30
                wreg(c.rx_ffe_pol1_addr,0,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 pol:0'% (lane_name_list[lane])
            elif tap_value[0]>ffe_k1_step:
                step = int(abs(tap_value[0])/(ffe_k3_step*2))
                ffe_taps(k3 = step_old+step*17,lane=lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 Pol:%d Step:%02X' % (lane_name_list[lane],ffe_k1_pol,step_old+step*17)

        if ffe_k4_pol==0:
            if tap_value[0]>81: # was 120: 2017-05-30
                wreg(c.rx_ffe_pol1_addr,1,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 pol:1'% (lane_name_list[lane])
            elif tap_value[0]<-ffe_k1_step:
                step = int(abs(tap_value[0])/(ffe_k4_step*2))
                ffe_taps(k4 = step_old+step*17,lane=lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 Pol:%d Step:%02X' % (lane_name_list[lane],ffe_k1_pol,step_old+step*17)
        else:
            if tap_value[0]<-81: # was -120: 017-05-30
                wreg(c.rx_ffe_pol1_addr,0,lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 pol:0'% (lane_name_list[lane])
            elif tap_value[0]>ffe_k1_step:
                step = int(abs(tap_value[0])/(ffe_k4_step*2))
                ffe_taps(k4 = step_old+step*17,lane=lane)
                if(print_en):print 'FFE Polarity Search: Lane %s, tap1 Pol:%d Step:%02X' % (lane_name_list[lane],ffe_k1_pol,step_old+step*17)

        ############################################################################################

        ffe_k1_pol = rreg(c.rx_ffe_pol1_addr,lane)
        ffe_k2_pol = rreg(c.rx_ffe_pol2_addr,lane)
        ffe_k3_pol = rreg(c.rx_ffe_pol3_addr,lane)
        ffe_k4_pol = rreg(c.rx_ffe_pol4_addr,lane)
        if(print_en):print("\nFFE Taps Pol Lane %s, After:(%2d,%2d,%2d,%2d)"%(lane_name_list[lane],ffe_k1_pol,ffe_k2_pol,ffe_k3_pol,ffe_k4_pol)),
        [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin] = ffe_taps(lane=lane)[lane]
        if(print_en):print("\nFFE Taps Val Lane %s, After:(%02X,%02X,%02X,%02X,%02X,%02X)" %(lane_name_list[lane],ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin))
        #if(print_en):print ('FFE Polarity Search: OptTime: %2.2f, '%(time.time()-start_time))
####################################################################################################
# 
# 
# Doing Gettapvalue for the target ISI tap and not all f2 to f5
#
####################################################################################################
def ffe_search_loop(lane=None, print_en=1,loops=5):
    for i in range(loops):
        ffe_search(lane,print_en)
        
####################################################################################################
# 
# 
# Doing Gettapvalue for the target ISI tap and not all f2 to f5
#
####################################################################################################
def ffe_search(lane=None, print_en=1):
    lanes = get_lane_list(lane)
    
    Slice=gSlice
    get_lane_mode(lanes) # update the Encoding modes of all lanes for this Slice
    
    start_time = time.time()
    iter=5
    tap_range = range(2,6) # ISI f2 to f5
    lane_cntr=0
    #nbias=0# = ffe_nbias(lane=lane)[0]
    
    for lane in lanes:
        if(print_en):print('\nFFE_Search: Lane %s, using ISI Taps f%d to f%d'%(lane_name_list[lane],tap_range[0],tap_range[-1])),
        [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin] = ffe_taps(lane=lane)[lane]
        print("\nSlice %d Lane %s (%s) FFE_Search Start : ffe_taps(%02X, %02X, %02X, %02X, %02X, %02X, lane=%d)" %(Slice, lane_name_list[lane],gEncodingMode[gSlice][lane][0].upper(),ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,lane)),
        #if(print_en):print('\nFFE_Search: Lane %s, ffe_taps(%02X, %02X, %02X, %02X, %02X, %02X, lane=%d)' %(lane_name_list[lane],ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,lane)),
        org_nbias =ffe_nbias(lane=lane)[0]
        ffe_all_pol = ffe_pol(ffe_pol1=None, ffe_pol2=None, ffe_pol3=None, ffe_pol4=None, lane=lane)
        ffe_k1_pol = ffe_all_pol[lane_cntr][0]
        ffe_k2_pol = ffe_all_pol[lane_cntr][1]
        ffe_k3_pol = ffe_all_pol[lane_cntr][2]
        ffe_k4_pol = ffe_all_pol[lane_cntr][3]
        ffe_k1_step = 20 # ISI f2 step
        ffe_k2_step = 10 # ISI f3 step
        ffe_k3_step = 5  # ISI f4 step
        ffe_k4_step = 5  # ISI f5 step
        
        ###################################################### k4 (looking at f5)
        time.sleep(0.01)
        tap_value = [0] #place for f5 value here  [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, [5], lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%4.0f        "%x for x in tap_value]),
        ffe_k4_main1 = Gray_Bin(rreg(c.rx_ffe_k4_msb_addr, lane))
        ffe_k4_main0 = Gray_Bin(rreg(c.rx_ffe_k4_lsb_addr, lane))
        step_old =((ffe_k4_main1 << 4) + ffe_k4_main0)&0xFF
        step_k4 = step_old * ((-1)**ffe_k4_pol)
        step = 1 
        if tap_value[0]>ffe_k4_step:
            step_k4_new = step_k4-step*0x11
        elif tap_value[0]<-ffe_k4_step:
            step_k4_new = step_k4+step*0x11
        else:
            step_k4_new = step_k4
        if step_k4_new > 0xff: step_k4_new =0xff
        elif step_k4_new <-0xff: step_k4_new =-0xff
        if step_k4_new>0:
            ffe_pol4=0
            step_new=abs(step_k4_new)
        elif step_k4_new<0:
            ffe_pol4=1
            step_new=abs(step_k4_new)
        else:
            ffe_pol4=ffe_k4_pol
            step_new=step_old
        ffe_taps(k4=step_new,lane=lane)
        ffe_pol(ffe_pol4=ffe_pol4,lane=lane)
        if(print_en):print(' f5(%4d), K4(0x%02X -> 0x%02X), Pol(%d->%d)'%(tap_value[0],step_old,step_new,ffe_k4_pol,ffe_pol4)),

        ###################################################### k3 (looking at f4)
        time.sleep(0.01)
        tap_value = [0] # place for f4 value here [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, [4], lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%4.0f        "%x for x in tap_value]),
        ffe_k3_main1 = Gray_Bin(rreg(c.rx_ffe_k3_msb_addr, lane))
        ffe_k3_main0 = Gray_Bin(rreg(c.rx_ffe_k3_lsb_addr, lane))
        step_old =((ffe_k3_main1 << 4) + ffe_k3_main0)&0xFF
        step_k3 = step_old * ((-1)**ffe_k3_pol)
        step = 1 
        if tap_value[0]>ffe_k3_step:
            step_k3_new = step_k3-step*0x11
        elif tap_value[0]<-ffe_k3_step:
            step_k3_new = step_k3+step*0x11
        else:
            step_k3_new = step_k3
        if step_k3_new > 0xff: step_k3_new =0xff
        elif step_k3_new <-0xff: step_k3_new =-0xff
        if step_k3_new>0:
            ffe_pol3_new=0
            step_new=abs(step_k3_new)
        elif step_k3_new<0:
            ffe_pol3_new=1
            step_new=abs(step_k3_new)
        else:
            ffe_pol3_new=ffe_k3_pol
            step_new=step_old
        ffe_taps(k3=step_new,lane=lane)
        ffe_pol(ffe_pol3=ffe_pol3_new,lane=lane)
        if(print_en):print(' f4(%4d), K3(0x%02X -> 0x%02X), Pol(%d->%d)'%(tap_value[0],step_old,step_new,ffe_k3_pol,ffe_pol3_new)),

        ###################################################### k2 (looking at f3)
        time.sleep(0.01)
        tap_value = [0,1] # place for f2 and f3 values here     [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, [2,3], lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%4.0f"%x for x in tap_value]),
        ffe_k2_main1 = Gray_Bin(rreg(c.rx_ffe_k2_msb_addr, lane))
        ffe_k2_main0 = Gray_Bin(rreg(c.rx_ffe_k2_lsb_addr, lane))
        step_old =((ffe_k2_main1 << 4) + ffe_k2_main0)&0xFF
        step_k2 = step_old * ((-1)**ffe_k2_pol)
        step = 1
        step = (abs(tap_value[1]) / ffe_k1_step)
        #print("...K2 Step size = 0x%02X..."%step),
        if tap_value[1]>ffe_k2_step:
            step_k2_new = step_k2-step*0x11
        elif tap_value[1]<-ffe_k2_step:
            step_k2_new = step_k2+step*0x11
        else:
            step_k2_new = step_k2
        if step_k2_new > 0xff: step_k2_new =0xff
        elif step_k2_new <-0xff: step_k2_new =-0xff
        if step_k2_new>0:
            ffe_pol2=0
            step_new=abs(step_k2_new)
        elif step_k2_new<0:
            ffe_pol2=1
            step_new=abs(step_k2_new)
        else: # no K2 polarity change
            ffe_pol2=ffe_k2_pol # original K2 polarity
            step_new=step_old
        ffe_taps(k2=step_new,lane=lane)
        ffe_pol(ffe_pol2=ffe_pol2,lane=lane)
        if(print_en):print(' f3(%4d), K2(0x%02X -> 0x%02X), Pol(%d->%d)'%(tap_value[1],step_old,step_new,ffe_k2_pol,ffe_pol2)),
        
        # K2-for-f3 adjustment done, if K2 pol changed, check f2 to see if f2 became too large
        if ffe_k2_pol != ffe_pol2:
            f2_prev=tap_value[0]
            f2_new = sw_pam4_isi (1, 0, [0], lane) # get most ISI tap f2
            if (abs(f2_new[0]) - abs(f2_prev) > 50): # if f2 changed so large
                    if(print_en):print('\nFFE_Search: Lane %s, '%(lane_name_list[lane],)),        
                    if(print_en):print(' K2 Polarity reverted back to orginal polarity'),
                    ffe_pol2=ffe_k2_pol
                    ffe_pol(ffe_pol2=ffe_pol2,lane=lane)
            
        
        ###################################################### k1 (looking at f2)
        time.sleep(0.01)
        tap_value = [0] # place for f2 value here   [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, [2], lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%4.0f        "%x for x in tap_value]),
        ffe_k1_main1 = Gray_Bin(rreg(c.rx_ffe_k1_msb_addr, lane))
        ffe_k1_main0 = Gray_Bin(rreg(c.rx_ffe_k1_lsb_addr, lane))
        step_old = ((ffe_k1_main1 << 4) + ffe_k1_main0)&0xFF
        nbias = ffe_nbias(lane=lane)[0]
        step = 1
        step = (abs(tap_value[0]) / ffe_k1_step)
        #print("...K1 Step size = 0x%02X..."%step),
        if tap_value[0]>ffe_k1_step: # if f2 too positive 
            if ffe_k1_pol==0: # if f2  positive, k1 pol positive
                if step_old>=0x11:  # if k1 > 0x11, inc k1 by 0x11
                    step_new=step_old-step*0x11
                    ffe_taps(k1=step_new,lane=lane)
                elif step_old>0: # if k1 0x01 to 0x11, stop
                    step_new=0
                    ffe_taps(k1=step_new,lane=lane)
                elif nbias>8:
                    next_nbias=nbias-2
                    ffe_nbias(nbias=next_nbias,lane=lane)
                elif nbias<6:
                    next_nbias=nbias+2
                    ffe_nbias(nbias=next_nbias,lane=lane)
                else:
                    ffe_k1_pol_new = 1
                    ffe_pol(ffe_pol1=ffe_k1_pol_new,lane=lane)
            else: # if f2  positive, k1 pol negative
                if step_old<=0xee:
                    step_new=step_old+step*0x11
                else:
                    step_new=0xff
                ffe_taps(k1=step_new,lane=lane)
        elif tap_value[0]<-ffe_k1_step: # if f2 too negative
            if ffe_k1_pol==1: # f2 too negative, k1 polarity is negative
                if step_old>=0x11:
                    step_new=step_old-step*0x11
                    ffe_taps(k1=step_new,lane=lane)
                elif step_old>0:
                    step_new=0
                    ffe_taps(k1=step_new,lane=lane)
                elif nbias>8:
                    next_nbias=nbias-2
                    ffe_nbias(nbias=next_nbias,lane=lane)
                elif nbias<6:
                    next_nbias=nbias+2
                    ffe_nbias(nbias=next_nbias,lane=lane)
                else:
                    ffe_k1_pol_new = 0
                    ffe_pol(ffe_pol1=ffe_k1_pol_new,lane=lane)
            else: # f2 too negative, k1 polarity is positive
                if step_old<=0xee:
                    step_new=step_old+step*0x11
                else:
                    step_new=0xff
                ffe_taps(k1=step_new,lane=lane)
        else:
            pass
        ffe_pol1 = ffe_pol(lane=lane)[0][0]
        ffe_k1_main1 = Gray_Bin(rreg(c.rx_ffe_k1_msb_addr, lane))
        ffe_k1_main0 = Gray_Bin(rreg(c.rx_ffe_k1_lsb_addr, lane))
        step_new = ((ffe_k1_main1 << 4) + ffe_k1_main0)&0xFF
        if(print_en):print(' f2(%4d), K1(0x%02X -> 0x%02X), Pol(%1d->%1d)'%(tap_value[0],step_old,step_new,ffe_k1_pol,ffe_pol1)),
        
        # N E W ################################################### k2 second time (looking at f3)
        time.sleep(0.01)
        tap_value = [0,1] # place for f2 and f3 values here     [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, [2,3], lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%4.0f"%x for x in tap_value]),
        ffe_k2_main1 = Gray_Bin(rreg(c.rx_ffe_k2_msb_addr, lane))
        ffe_k2_main0 = Gray_Bin(rreg(c.rx_ffe_k2_lsb_addr, lane))
        step_old =((ffe_k2_main1 << 4) + ffe_k2_main0)&0xFF
        step_k2 = step_old * ((-1)**ffe_k2_pol)
        step = 1
        step = (abs(tap_value[1]) / ffe_k1_step)
        #print("...K2 Step size = 0x%02X..."%step),
        if tap_value[1]>ffe_k2_step:
            step_k2_new = step_k2-step*0x11
        elif tap_value[1]<-ffe_k2_step:
            step_k2_new = step_k2+step*0x11
        else:
            step_k2_new = step_k2
        if step_k2_new > 0xff: step_k2_new =0xff
        elif step_k2_new <-0xff: step_k2_new =-0xff
        if step_k2_new>0:
            ffe_pol2=0
            step_new=abs(step_k2_new)
        elif step_k2_new<0:
            ffe_pol2=1
            step_new=abs(step_k2_new)
        else: # no K2 polarity change
            ffe_pol2=ffe_k2_pol # original K2 polarity
            step_new=step_old
        ffe_taps(k2=step_new,lane=lane)
        ffe_pol(ffe_pol2=ffe_pol2,lane=lane)
        if(print_en):print(' f3(%4d), K2(0x%02X -> 0x%02X), Pol(%d->%d)'%(tap_value[1],step_old,step_new,ffe_k2_pol,ffe_pol2)),
        
        # K2-for-f3 adjustment done, if K2 pol changed, check f2 to see if f2 became too large
        if ffe_k2_pol != ffe_pol2:
            f2_prev=tap_value[0]
            f2_new = sw_pam4_isi (1, 0, [0], lane) # get most ISI tap f2
            if (abs(f2_new[0]) - abs(f2_prev) > 50): # if f2 changed so large
                    if(print_en):print('\nFFE_Search: Lane %s, '%(lane_name_list[lane],)),        
                    if(print_en):print(' K2 Polarity reverted back to orginal polarity'),
                    ffe_pol2=ffe_k2_pol
                    ffe_pol(ffe_pol2=ffe_pol2,lane=lane)
            
        new_nbias = ffe_nbias(lane=lane)[0]
        if(print_en):print("\nFFE_Search: Lane %s, nbias(%d -> %d)"%(lane_name_list[lane],org_nbias,new_nbias)),
        
        [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin] = ffe_taps(lane=lane)[lane]
        print("\nSlice %d Lane %s (%s) FFE_Search Finish: ffe_taps(%02X, %02X, %02X, %02X, %02X, %02X, lane=%d)" %(Slice, lane_name_list[lane],gEncodingMode[gSlice][lane][0].upper(),ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,lane)),
        lane_cntr+=1
####################################################################################################
# 
# 
####################################################################################################
def ffe_nbias_main_a1(nbias=None,lane=8):
    nbias_main_addr = [0x1d6,[15,12]] ############ need add into 'xxx_reg.py' !!!
    if nbias==None:
        ffe_nbias=Gray_Bin(rreg(nbias_main_addr,lane))
        return ffe_nbias
    else:
        wreg(nbias_main_addr,Bin_Gray(nbias),lane)

def ffe_pol_a1(ffe_pol1=None, ffe_pol2=None, ffe_pol3=None, ffe_pol4=None, lane=None):

    lanes = get_lane_list(lane)
         
    for lane in lanes:
        if ffe_pol1!=None: wreg(c.rx_ffe_pol1_addr,ffe_pol1,lane)
        if ffe_pol2!=None: wreg(c.rx_ffe_pol2_addr,ffe_pol2,lane)
        if ffe_pol3!=None: wreg(c.rx_ffe_pol3_addr,ffe_pol3,lane)
        if ffe_pol4!=None: wreg(c.rx_ffe_pol4_addr,ffe_pol4,lane)
         
        ffe_pol1 = rreg(c.rx_ffe_pol1_addr,lane)
        ffe_pol2 = rreg(c.rx_ffe_pol2_addr,lane)
        ffe_pol3 = rreg(c.rx_ffe_pol3_addr,lane)
        ffe_pol4 = rreg(c.rx_ffe_pol4_addr,lane)
        
    return ffe_pol1, ffe_pol2, ffe_pol3, ffe_pol4
    
####################################################################################################
# 
# 
# Doing Gettapvalue for the target ISI tap and not all f2 to f5
#
####################################################################################################
def ffe_search_a1(lane=None, print_en=1):

    lanes = get_lane_list(lane)

    Slice=gSlice
    start_time = time.time()
    iter=5
    tap_range = range(2,6) # ISI f2 to f5
    lane_cntr=0
    
    for lane in lanes:
        if(print_en):print('\nFFE_Search_a1: Lane %s, using ISI Taps %d to %d'%(lane_name_list[lane],tap_range[0],tap_range[-1])),
        [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin] = ffe_taps(lane=lane)[lane]#[0]
        print("\nSlice %d Lane %s (%s) FFE_Search Start : ffe_taps(%02X, %02X, %02X, %02X, %02X, %02X, lane=%d)" %(Slice, lane_name_list[lane],gEncodingMode[gSlice][lane][0].upper(),ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,lane)),

        ffe_all_pol = ffe_pol_a1(ffe_pol1=None, ffe_pol2=None, ffe_pol3=None, ffe_pol4=None, lane=lane)
        ffe_k1_pol = ffe_all_pol[0]
        ffe_k2_pol = ffe_all_pol[1]
        ffe_k3_pol = ffe_all_pol[2]
        ffe_k4_pol = ffe_all_pol[3]
        ffe_k1_step = 20 # ISI f2 step
        ffe_k2_step = 10 # ISI f3 step
        ffe_k3_step = 5  # ISI f4 step
        ffe_k4_step = 5  # ISI f5 step
        
        ###################################################### k4
        time.sleep(0.01)
        tap_value = [0] #place for f5 value here  [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, [5], lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search_a1: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%4.0f"%x for x in tap_value]),
        ffe_k4_main1 = Gray_Bin(rreg(c.rx_ffe_k4_msb_addr, lane))
        ffe_k4_main0 = Gray_Bin(rreg(c.rx_ffe_k4_lsb_addr, lane))
        step_old =((ffe_k4_main1 << 4) + ffe_k4_main0)&0xFF
        step_k4 = step_old * ((-1)**ffe_k4_pol)
        step = 4
        if ffe_k4_step*9>=abs(tap_value[0]): step = 3
        if ffe_k4_step*7>=abs(tap_value[0]): step = 2
        if ffe_k4_step*5>=abs(tap_value[0]): step = 1

        if tap_value[0]>ffe_k4_step:
            step_k4_new = step_k4-step*0x11
        elif tap_value[0]<-ffe_k4_step:
            step_k4_new = step_k4+step*0x11
        else:
            step_k4_new = step_k4
        if step_k4_new > 0xff: step_k4_new =0xff
        elif step_k4_new <-0xff: step_k4_new =-0xff

        if step_k4_new>0 and ffe_k4_pol==0:
            ffe_pol4=0
            step_new=abs(step_k4_new)
        elif step_k4_new<0 and ffe_k4_pol==1:
            ffe_pol4=1
            step_new=abs(step_k4_new)

        elif step_k4_new>0 and ffe_k4_pol==1:
            if step_old == 1:
                ffe_pol4=0
                step_new=0
            else:
                ffe_pol4=1
                step_new=0
        elif step_k4_new<0 and ffe_k4_pol==0:
            if step_old == 1:
                ffe_pol4=1
                step_new=0
            else:                
                ffe_pol4=0
                step_new=0

        elif step_k4_new==0:
            ffe_pol4=ffe_k4_pol
            step_new=0
        else:
            ffe_pol4=ffe_k4_pol
            step_new=step_old

        if step_new==0:step_new=1
        ffe_taps(k4=step_new,lane=lane)
        ffe_pol_a1(ffe_pol4=ffe_pol4,lane=lane)
        if(print_en):print(' f5(%4d), K4(0x%02X -> 0x%02X), Pol(%d->%d)'%(tap_value[0],step_old,step_new,ffe_k4_pol,ffe_pol4)),

        ###################################################### k3 (looking at f4)
        time.sleep(0.01)
        tap_value = [0] # place for f4 value here [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, [4], lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search_a1: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%4.0f"%x for x in tap_value]),
        ffe_k3_main1 = Gray_Bin(rreg(c.rx_ffe_k3_msb_addr, lane))
        ffe_k3_main0 = Gray_Bin(rreg(c.rx_ffe_k3_lsb_addr, lane))
        step_old =((ffe_k3_main1 << 4) + ffe_k3_main0)&0xFF
        step_k3 = step_old * ((-1)**ffe_k3_pol)
        step = 4
        if ffe_k3_step*9>=abs(tap_value[0]): step = 3
        if ffe_k3_step*7>=abs(tap_value[0]): step = 2
        if ffe_k3_step*5>=abs(tap_value[0]): step = 1

        if tap_value[0]>ffe_k3_step:
            step_k3_new = step_k3-step*0x11
        elif tap_value[0]<-ffe_k3_step:
            step_k3_new = step_k3+step*0x11
        else:
            step_k3_new = step_k3
        if step_k3_new > 0xff: step_k3_new =0xff
        elif step_k3_new <-0xff: step_k3_new =-0xff

        if step_k3_new>0 and ffe_k3_pol==0:
            ffe_pol3=0
            step_new=abs(step_k3_new)
        elif step_k3_new<0 and ffe_k3_pol==1:
            ffe_pol3=1
            step_new=abs(step_k3_new)

        elif step_k3_new>0 and ffe_k3_pol==1:
            if step_old == 1:
                ffe_pol3=0
                step_new=0
            else:
                ffe_pol3=1
                step_new=0
        elif step_k3_new<0 and ffe_k3_pol==0:
            if step_old == 1:
                ffe_pol3=1
                step_new=0
            else:                
                ffe_pol3=0
                step_new=0

        elif step_k3_new==0:
            ffe_pol3=ffe_k3_pol
            step_new=0
        else:
            ffe_pol3=ffe_k3_pol
            step_new=step_old
        
        if step_new==0:step_new=1
        ffe_taps(k3=step_new,lane=lane)
        ffe_pol_a1(ffe_pol3=ffe_pol3,lane=lane)
        if(print_en):print(' f4(%4d), K3(0x%02X -> 0x%02X), Pol(%d->%d)'%(tap_value[0],step_old,step_new,ffe_k3_pol,ffe_pol3)),

        ###################################################### k2 (looking at f3)
        time.sleep(0.01)
        tap_value = [0] # place for f3 values here     [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, [3], lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search_a1: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%4.0f"%x for x in tap_value]),
        ffe_k2_main1 = Gray_Bin(rreg(c.rx_ffe_k2_msb_addr, lane))
        ffe_k2_main0 = Gray_Bin(rreg(c.rx_ffe_k2_lsb_addr, lane))
        step_old =((ffe_k2_main1 << 4) + ffe_k2_main0)&0xFF
        step_k2 = step_old * ((-1)**ffe_k2_pol)
        step = 4
        if ffe_k2_step*9>=abs(tap_value[0]): step = 3
        if ffe_k2_step*7>=abs(tap_value[0]): step = 2
        if ffe_k2_step*5>=abs(tap_value[0]): step = 1

        if tap_value[0]>ffe_k2_step:
            step_k2_new = step_k2-step*0x11
        elif tap_value[0]<-ffe_k2_step:
            step_k2_new = step_k2+step*0x11
        else:
            step_k2_new = step_k2
        if step_k2_new > 0xff: step_k2_new =0xff
        elif step_k2_new <-0xff: step_k2_new =-0xff


        if step_k2_new>0 and ffe_k2_pol==0:
            ffe_pol2=0
            step_new=abs(step_k2_new)
        elif step_k2_new<0 and ffe_k2_pol==1:
            ffe_pol2=1
            step_new=abs(step_k2_new)

        elif step_k2_new>0 and ffe_k2_pol==1:
            if step_old == 1:
                ffe_pol2=0
                step_new=0
            else:
                ffe_pol2=1
                step_new=0
        elif step_k2_new<0 and ffe_k2_pol==0:
            if step_old == 1:
                ffe_pol2=1
                step_new=0
            else:                
                ffe_pol2=0
                step_new=0

        elif step_k2_new==0:
            ffe_pol2=ffe_k2_pol
            step_new=0
        else:
            ffe_pol2=ffe_k2_pol
            step_new=step_old

        if step_new==0:step_new=1
        ffe_taps(k2=step_new,lane=lane)
        ffe_pol_a1(ffe_pol2=ffe_pol2,lane=lane)
        if(print_en):print(' f3(%4d), K2(0x%02X -> 0x%02X), Pol(%d->%d)'%(tap_value[0],step_old,step_new,ffe_k2_pol,ffe_pol2)),


        ###################################################### k1 (looking at f2)
        time.sleep(0.01)
        tap_value = [0] # place for f2 value here   [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, [2], lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search_a1: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%4.0f"%x for x in tap_value]),
        ffe_k1_main1 = Gray_Bin(rreg(c.rx_ffe_k1_msb_addr, lane))
        ffe_k1_main0 = Gray_Bin(rreg(c.rx_ffe_k1_lsb_addr, lane))
        step_old = ((ffe_k1_main1 << 4) + ffe_k1_main0)&0xFF
        nbias = ffe_nbias_main_a1(lane=lane)
        step = 4
        if ffe_k1_step*9>=abs(tap_value[0]): step = 3
        if ffe_k1_step*7>=abs(tap_value[0]): step = 2
        if ffe_k1_step*5>=abs(tap_value[0]): step = 1

 
        if tap_value[0]>ffe_k1_step:
            if ffe_k1_pol==0:
                if step_old>=0x11:
                    step_new=step_old-step*0x11
                    if step_new==0:step_new=1
                    ffe_taps(k1=step_new,lane=lane)
                elif step_old>1:
                    step_new=1
                    if step_new==0:step_new=1
                    ffe_taps(k1=step_new,lane=lane)                
                elif nbias<=6 or nbias>=8:
                    ffe_nbias=7
                    ffe_nbias_main_a1(nbias=ffe_nbias,lane=lane)
                    if abs(tap_value[0])>=ffe_k1_step*2:
                        ffe_k1_pol_new = 1
                        ffe_pol_a1(ffe_pol1=ffe_k1_pol_new,lane=lane)
                else:
                    ffe_k1_pol_new = 1
                    ffe_pol_a1(ffe_pol1=ffe_k1_pol_new,lane=lane)
            else:
                if step_old<=0xee:
                    step_new=step_old+step*0x11
                else:
                    step_new=0xff
                if step_new==0:step_new=1
                ffe_taps(k1=step_new,lane=lane)
        elif tap_value[0]<-ffe_k1_step:
            if ffe_k1_pol==1:
                if step_old>=0x11:
                    step_new=step_old-step*0x11
                    if step_new==0:step_new=1
                    ffe_taps(k1=step_new,lane=lane)
                elif step_old>1:
                    step_new=1
                    ffe_taps(k1=step_new,lane=lane)
                elif nbias<=6 or nbias>=8:
                    ffe_nbias=7
                    ffe_nbias_main_a1(nbias=ffe_nbias,lane=lane)
                    if abs(tap_value[0])>=ffe_k1_step*2:
                        ffe_k1_pol_new = 0
                        ffe_pol_a1(ffe_pol1=ffe_k1_pol_new,lane=lane)
                else:
                    ffe_k1_pol_new = 0
                    ffe_pol_a1(ffe_pol1=ffe_k1_pol_new,lane=lane)               
            else:
                if step_old<=0xee:
                    step_new=step_old+step*0x11
                else:
                    step_new=0xff
                if step_new==0:step_new=1
                ffe_taps(k1=step_new,lane=lane)
        else:
            pass
        ffe_pol1= ffe_pol_a1(lane=lane)[0]
        ffe_k1_main1 = Gray_Bin(rreg(c.rx_ffe_k1_msb_addr, lane))
        ffe_k1_main0 = Gray_Bin(rreg(c.rx_ffe_k1_lsb_addr, lane))
        step_new = ((ffe_k1_main1 << 4) + ffe_k1_main0)&0xFF
        if(print_en):print(' f2(%4d), K1(0x%02X -> 0x%02X), Pol(%1d->%1d)'%(tap_value[0],step_old,step_new,ffe_k1_pol,ffe_pol1)),


        [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin] = ffe_taps(lane=lane)[lane]
        print("\nSlice %d Lane %s (%s) FFE_Search Finish: ffe_taps(%02X, %02X, %02X, %02X, %02X, %02X, lane=%d)" %(Slice, lane_name_list[lane],gEncodingMode[gSlice][lane][0].upper(),ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,lane)),

####################################################################################################
# 
# 
# Doing Gettapvalue for the target ISI tap and not all f2 to f5
#
####################################################################################################
def ffe_search_a1_orig(lane=None, print_en=1):

    lanes = get_lane_list(lane)

    start_time = time.time()
    iter=5
    tap_range = range(2,6) # ISI f2 to f5
    lane_cntr=0
    
    for lane in lanes:
        if(print_en):print('\nFFE_Search_a1: Lane %s, using ISI Taps %d to %d'%(lane_name_list[lane],tap_range[0],tap_range[-1])),
        [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin] = ffe_taps(lane=lane)[lane]#[0]
        if(print_en):print('\nFFE_Search_a1: Lane %s, Taps:(k1-%02X,k2-%02X,k3-%02X,k4-%02X,s1-%02X,s2-%02X)' %(lane_name_list[lane],ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin)),

        ffe_all_pol = ffe_pol_a1(ffe_pol1=None, ffe_pol2=None, ffe_pol3=None, ffe_pol4=None, lane=lane)
        ffe_k1_pol = ffe_all_pol[0]
        ffe_k2_pol = ffe_all_pol[1]
        ffe_k3_pol = ffe_all_pol[2]
        ffe_k4_pol = ffe_all_pol[3]
        ffe_k1_step = 20 # ISI f2 step
        ffe_k2_step = 10 # ISI f3 step
        ffe_k3_step = 5  # ISI f4 step
        ffe_k4_step = 5  # ISI f5 step
        
        ###################################################### k4
        time.sleep(0.01)
        tap_value = [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, tap_range, lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search_a1: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%6.1f"%x for x in tap_value]),
        ffe_k4_main1 = Gray_Bin(rreg(c.rx_ffe_k4_msb_addr, lane))
        ffe_k4_main0 = Gray_Bin(rreg(c.rx_ffe_k4_lsb_addr, lane))
        step_old =((ffe_k4_main1 << 4) + ffe_k4_main0)&0xFF
        step_k4 = step_old * ((-1)**ffe_k4_pol)

        step = 4
        if ffe_k4_step*9>=abs(tap_value[3]): step = 3
        if ffe_k4_step*7>=abs(tap_value[3]): step = 2
        if ffe_k4_step*5>=abs(tap_value[3]): step = 1

        if tap_value[3]>ffe_k4_step:
            step_k4_new = step_k4-step*0x11
        elif tap_value[3]<-ffe_k4_step:
            step_k4_new = step_k4+step*0x11
        else:
            step_k4_new = step_k4
        if step_k4_new > 0xff: step_k4_new =0xff
        elif step_k4_new <-0xff: step_k4_new =-0xff

        if step_k4_new>0 and ffe_k4_pol==0:
            ffe_pol4=0
            step_new=abs(step_k4_new)
        elif step_k4_new<0 and ffe_k4_pol==1:
            ffe_pol4=1
            step_new=abs(step_k4_new)

        elif step_k4_new>0 and ffe_k4_pol==1:
            if step_old == 1:
                ffe_pol4=0
                step_new=0
            else:
                ffe_pol4=1
                step_new=0
        elif step_k4_new<0 and ffe_k4_pol==0:
            if step_old == 1:
                ffe_pol4=1
                step_new=0
            else:                
                ffe_pol4=0
                step_new=0

        elif step_k4_new==0:
            ffe_pol4=ffe_k4_pol
            step_new=0
        else:
            ffe_pol4=ffe_k4_pol
            step_new=step_old

        if step_new==0:step_new=1
        ffe_taps(k4=step_new,lane=lane)
        ffe_pol_a1(ffe_pol4=ffe_pol4,lane=lane)
        if(print_en):print(' tap4 -> val(%4d), Pol(%d->%d), step(%02X->%02X)'%(tap_value[3],ffe_k4_pol,ffe_pol4,step_old,step_new)),

        ###################################################### k3
        time.sleep(0.01)
        tap_value = [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, tap_range, lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search_a1: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%6.1f"%x for x in tap_value]),
        ffe_k3_main1 = Gray_Bin(rreg(c.rx_ffe_k3_msb_addr, lane))
        ffe_k3_main0 = Gray_Bin(rreg(c.rx_ffe_k3_lsb_addr, lane))
        step_old =((ffe_k3_main1 << 4) + ffe_k3_main0)&0xFF
        step_k3 = step_old * ((-1)**ffe_k3_pol)

        step = 4
        if ffe_k3_step*9>=abs(tap_value[2]): step = 3
        if ffe_k3_step*7>=abs(tap_value[2]): step = 2
        if ffe_k3_step*5>=abs(tap_value[2]): step = 1

        if tap_value[2]>ffe_k3_step:
            step_k3_new = step_k3-step*0x11
        elif tap_value[2]<-ffe_k3_step:
            step_k3_new = step_k3+step*0x11
        else:
            step_k3_new = step_k3
        if step_k3_new > 0xff: step_k3_new =0xff
        elif step_k3_new <-0xff: step_k3_new =-0xff

        if step_k3_new>0 and ffe_k3_pol==0:
            ffe_pol3=0
            step_new=abs(step_k3_new)
        elif step_k3_new<0 and ffe_k3_pol==1:
            ffe_pol3=1
            step_new=abs(step_k3_new)

        elif step_k3_new>0 and ffe_k3_pol==1:
            if step_old == 1:
                ffe_pol3=0
                step_new=0
            else:
                ffe_pol3=1
                step_new=0
        elif step_k3_new<0 and ffe_k3_pol==0:
            if step_old == 1:
                ffe_pol3=1
                step_new=0
            else:                
                ffe_pol3=0
                step_new=0

        elif step_k3_new==0:
            ffe_pol3=ffe_k3_pol
            step_new=0
        else:
            ffe_pol3=ffe_k3_pol
            step_new=step_old
        
        if step_new==0:step_new=1
        ffe_taps(k3=step_new,lane=lane)
        ffe_pol_a1(ffe_pol3=ffe_pol3,lane=lane)
        if(print_en):print(' tap3 -> val(%4d), Pol(%d->%d), step(%02X->%02X)'%(tap_value[2],ffe_k3_pol,ffe_pol3,step_old,step_new)),

        ###################################################### k2
        time.sleep(0.01)
        tap_value = [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, tap_range, lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search_a1: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%6.1f"%x for x in tap_value]),
        ffe_k2_main1 = Gray_Bin(rreg(c.rx_ffe_k2_msb_addr, lane))
        ffe_k2_main0 = Gray_Bin(rreg(c.rx_ffe_k2_lsb_addr, lane))
        step_old =((ffe_k2_main1 << 4) + ffe_k2_main0)&0xFF
        step_k2 = step_old * ((-1)**ffe_k2_pol)

        step = 4
        if ffe_k2_step*9>=abs(tap_value[1]): step = 3
        if ffe_k2_step*7>=abs(tap_value[1]): step = 2
        if ffe_k2_step*5>=abs(tap_value[1]): step = 1

        if tap_value[1]>ffe_k2_step:
            step_k2_new = step_k2-step*0x11
        elif tap_value[1]<-ffe_k2_step:
            step_k2_new = step_k2+step*0x11
        else:
            step_k2_new = step_k2
        if step_k2_new > 0xff: step_k2_new =0xff
        elif step_k2_new <-0xff: step_k2_new =-0xff


        if step_k2_new>0 and ffe_k2_pol==0:
            ffe_pol2=0
            step_new=abs(step_k2_new)
        elif step_k2_new<0 and ffe_k2_pol==1:
            ffe_pol2=1
            step_new=abs(step_k2_new)

        elif step_k2_new>0 and ffe_k2_pol==1:
            if step_old == 1:
                ffe_pol2=0
                step_new=0
            else:
                ffe_pol2=1
                step_new=0
        elif step_k2_new<0 and ffe_k2_pol==0:
            if step_old == 1:
                ffe_pol2=1
                step_new=0
            else:                
                ffe_pol2=0
                step_new=0

        elif step_k2_new==0:
            ffe_pol2=ffe_k2_pol
            step_new=0
        else:
            ffe_pol2=ffe_k2_pol
            step_new=step_old

        if step_new==0:step_new=1
        ffe_taps(k2=step_new,lane=lane)
        ffe_pol_a1(ffe_pol2=ffe_pol2,lane=lane)
        if(print_en):print(' tap2 -> val(%4d), Pol(%d->%d), step(%02X->%02X)'%(tap_value[1],ffe_k2_pol,ffe_pol2,step_old,step_new)),


        ###################################################### k1
        time.sleep(0.01)
        tap_value = [0]*len(tap_range)
        for i in range(iter):
            tap_list = sw_pam4_isi (1, 0, tap_range, lane)
            tap_value = map(lambda x,y:x+(y/iter), tap_value, tap_list)
        if(print_en):print('\nFFE_Search_a1: Lane %s, '%(lane_name_list[lane],)),
        if(print_en):print(["%6.1f"%x for x in tap_value]),
        ffe_k1_main1 = Gray_Bin(rreg(c.rx_ffe_k1_msb_addr, lane))
        ffe_k1_main0 = Gray_Bin(rreg(c.rx_ffe_k1_lsb_addr, lane))
        step_old = ((ffe_k1_main1 << 4) + ffe_k1_main0)&0xFF
        nbias = ffe_nbias_main_a1(lane=lane)
        
        step = 4
        if ffe_k1_step*9>=abs(tap_value[0]): step = 3
        if ffe_k1_step*7>=abs(tap_value[0]): step = 2
        if ffe_k1_step*5>=abs(tap_value[0]): step = 1

 
        if tap_value[0]>ffe_k1_step:
            if ffe_k1_pol==0:
                if step_old>=0x11:
                    step_new=step_old-step*0x11
                    if step_new==0:step_new=1
                    ffe_taps(k1=step_new,lane=lane)
                elif step_old>1:
                    step_new=1
                    if step_new==0:step_new=1
                    ffe_taps(k1=step_new,lane=lane)                
                elif nbias<=6 or nbias>=8:
                    ffe_nbias=7
                    ffe_nbias_main_a1(nbias=ffe_nbias,lane=lane)
                    if abs(tap_value[0])>=ffe_k1_step*2:
                        ffe_k1_pol_new = 1
                        ffe_pol_a1(ffe_pol1=ffe_k1_pol_new,lane=lane)
                else:
                    ffe_k1_pol_new = 1
                    ffe_pol_a1(ffe_pol1=ffe_k1_pol_new,lane=lane)
            else:
                if step_old<=0xee:
                    step_new=step_old+step*0x11
                else:
                    step_new=0xff
                if step_new==0:step_new=1
                ffe_taps(k1=step_new,lane=lane)
        elif tap_value[0]<-ffe_k1_step:
            if ffe_k1_pol==1:
                if step_old>=0x11:
                    step_new=step_old-step*0x11
                    if step_new==0:step_new=1
                    ffe_taps(k1=step_new,lane=lane)
                elif step_old>1:
                    step_new=1
                    ffe_taps(k1=step_new,lane=lane)
                elif nbias<=6 or nbias>=8:
                    ffe_nbias=7
                    ffe_nbias_main_a1(nbias=ffe_nbias,lane=lane)
                    if abs(tap_value[0])>=ffe_k1_step*2:
                        ffe_k1_pol_new = 0
                        ffe_pol_a1(ffe_pol1=ffe_k1_pol_new,lane=lane)
                else:
                    ffe_k1_pol_new = 0
                    ffe_pol_a1(ffe_pol1=ffe_k1_pol_new,lane=lane)               
            else:
                if step_old<=0xee:
                    step_new=step_old+step*0x11
                else:
                    step_new=0xff
                if step_new==0:step_new=1
                ffe_taps(k1=step_new,lane=lane)
        else:
            pass
        ffe_pol1= ffe_pol_a1(lane=lane)[0]
        ffe_k1_main1 = Gray_Bin(rreg(c.rx_ffe_k1_msb_addr, lane))
        ffe_k1_main0 = Gray_Bin(rreg(c.rx_ffe_k1_lsb_addr, lane))
        step_new = ((ffe_k1_main1 << 4) + ffe_k1_main0)&0xFF
        if(print_en):print(' tap1 -> val(%4d), Pol(%1d->%1d), step(%02X->%02X), nbias(%3d->%3d)'%(tap_value[0],ffe_k1_pol,ffe_pol1,step_old,step_new,nbias,ffe_nbias_main_a1(lane=lane))),


        [ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin,ffe_sf_bin] = ffe_taps(lane=lane)[lane]
        if(print_en):print('\nFFE_Search_a1: Lane %s, Taps:(k1-%02X,k2-%02X,k3-%02X,k4-%02X,s1-%02X,s2-%02X)' %(lane_name_list[lane],ffe_k1_bin,ffe_k2_bin,ffe_k3_bin,ffe_k4_bin,ffe_s1_bin,ffe_s2_bin)),

####################################################################################################
# Initialize FEC Analyzer function
#
# This is a per-lane function
####################################################################################################
def fec_ana_init(lane=None, delay=.1, err_type=0, T=15, M=10, N=5440, print_en=1):
    lanes = get_lane_list(lane)

    FEC_ANA_BASE_ADDR = 0x1C0
    global gFecThresh
    gFecThresh=T
        
    for ln in lanes:         
        if gEncodingMode[gSlice][ln][0].lower() != 'pam4':
            if T>7: T=7 ## if lane is in NRZ mode, max T is 7 

        #err_type: tei ctrl teo ctrl
        wreg(FEC_ANA_BASE_ADDR+0x9, 0x6202,lane=ln) #set PRBS mode: force reload
        wreg(FEC_ANA_BASE_ADDR+0x8, ( rreg(FEC_ANA_BASE_ADDR+0x8,lane=ln) | 0x0018), lane=ln) #set PRBS mode:PRBS31
        wreg(FEC_ANA_BASE_ADDR+0x0, ((rreg(FEC_ANA_BASE_ADDR+0x0,lane=ln) & 0xFFF0) | M),lane=ln)  #set M = 10
        wreg(FEC_ANA_BASE_ADDR+0x4, N,lane=ln) #set N = 5440
        wreg(FEC_ANA_BASE_ADDR+0x5, (rreg(FEC_ANA_BASE_ADDR+0x5,lane=ln) & 0xf007) | ((T<<7)+(T<<3)),lane=ln) #set T = 2
        wreg(FEC_ANA_BASE_ADDR+0x1, 0x000b,  lane=ln) #reset FEC_counter
        wreg(FEC_ANA_BASE_ADDR+0x1, 0x0003,  lane=ln) #release the reset 
        wreg(FEC_ANA_BASE_ADDR+0xc, err_type,lane=ln) #set TEO error type
        wreg(FEC_ANA_BASE_ADDR+0xb, err_type,lane=ln) #set TEi error type
        if print_en: print '\n....Lane %s: FEC Analyzer Initialized' %(lane_name_list[ln]),
        
    time.sleep(delay)
####################################################################################################
def fec_ana_read(lane=None):
    lanes = get_lane_list(lane)

    print ('\n....FEC Analyzer Status....'),
    print ('\nLane, RAW Errors, Uncorr Frames, Uncorr Symbols, Uncorr PRBS'),
    for ln in lanes:         
        tei = fec_ana_tei(lane=ln)
        teo = fec_ana_teo(lane=ln)
        #sei = fec_ana_sei(lane=ln)
        #bei = fec_ana_bei(lane=ln)
        #print ('\n%4s, %10d, %13d, %14d, %11d'%(lane_name_list[ln],tei,teo,sei,bei)),
        print ('\n%4s, %10d, %13d'%(lane_name_list[ln],tei,teo)),
    
####################################################################################################
def fec_ana_tei(lane=None):
    FEC_ANA_BASE_ADDR = 0x1C0
    if lane==None: lane=gLane
   
    wreg(FEC_ANA_BASE_ADDR+0xD,4,lane) #set reading data of TEi low 16 bit
    tei_l = rreg(FEC_ANA_BASE_ADDR+0x7,lane)#read data
    wreg(FEC_ANA_BASE_ADDR+0xD,5,lane) #set reading data of TEi high 16 bit
    tei_h = rreg(FEC_ANA_BASE_ADDR+0x7,lane)#read data
    tei = tei_h*65536+tei_l #combinate the data
    #print '\n....Lane %s: TEi counter: %d' %(lane_name_list[lane],tei),
    return tei 

def fec_ana_teo(lane=None):
    FEC_ANA_BASE_ADDR = 0x1C0
    if lane==None: lane=gLane
    wreg(FEC_ANA_BASE_ADDR+0xD,6,lane ) #set reading data of TEo low 16 bit
    teo_l = rreg(FEC_ANA_BASE_ADDR+0x7,lane)#read data
    wreg(FEC_ANA_BASE_ADDR+0xD,7,lane) #set reading data of TEo high 16 bit
    teo_h = rreg(FEC_ANA_BASE_ADDR+0x7,lane)#read data
    teo = teo_h*65536+teo_l#combinate the data
    #print '\n....Lane %s: TEo counter: %d' %(lane_name_list[lane],teo),
    return teo 

def fec_ana_sei(lane=None):
    FEC_ANA_BASE_ADDR = 0x1C0
    if lane==None: lane=gLane
    wreg(FEC_ANA_BASE_ADDR+0xD,0,lane) #set reading data of SEi low 16 bit
    sei_l = rreg(FEC_ANA_BASE_ADDR+0x7,lane)#read data
    wreg(FEC_ANA_BASE_ADDR+0xD,1 ) #set reading data of SEi high 16 bit
    sei_h = rreg(FEC_ANA_BASE_ADDR+0x7,lane)#read data
    sei = sei_h*65536+sei_l#combinate the data
    #print '\n....Lane %s: SEi counter: %d' %(lane_name_list[lane],sei),
    return sei 

def fec_ana_bei(lane=None):
    FEC_ANA_BASE_ADDR = 0x1C0
    if lane==None: lane=gLane
    wreg(FEC_ANA_BASE_ADDR+0xD,2,lane) #set reading data of BEi low 16 bit
    bei_l = rreg(FEC_ANA_BASE_ADDR+0x7,lane)#read data
    wreg(FEC_ANA_BASE_ADDR+0xD,3 ,lane) #set reading data of BEi high 16 bit
    bei_h = rreg(FEC_ANA_BASE_ADDR+0x7,lane)#read data
    bei = bei_h*65536+bei_l#combinate the data
    #print '\n....Lane %s: BEi counter: %d' %(lane_name_list[lane],bei),
    return bei
    
def fec_ana_hist_one_bin_grp_setup(bin_group, err_type, T =2,M=10,N=5440,lane=None):
    lanes = get_lane_list(lane)
    FEC_ANA_BASE_ADDR = 0x1C0
    for ln in lanes:
        wreg([FEC_ANA_BASE_ADDR+0x5,[12]],1,ln)     #reset Histogram
        wreg([FEC_ANA_BASE_ADDR+0x5,[12]],0,ln)     #reset Histogram
        wreg([FEC_ANA_BASE_ADDR+0x5,[2,0]], bin_group, ln)  #select which group of histogram to read

        wreg(FEC_ANA_BASE_ADDR+0x9, 0x6202,ln) #set PRBS mode: force reload
        wreg(FEC_ANA_BASE_ADDR+0x8, (rreg(FEC_ANA_BASE_ADDR+0x8,ln) | 0x0018),ln) #set PRBS mode:PRBS31
        wreg(FEC_ANA_BASE_ADDR+0x0, ((rreg(FEC_ANA_BASE_ADDR+0x0,ln)&0xFFF0) | M),ln)  #set M = 10
        wreg(FEC_ANA_BASE_ADDR+0x4, N,ln) #set N = 5440
        wreg(FEC_ANA_BASE_ADDR+0x5, (rreg(FEC_ANA_BASE_ADDR+0x5,ln) & 0xf007) | ((T<<7)+(T<<3)) ,ln) #set T = 2
        wreg(FEC_ANA_BASE_ADDR+0x1, 0x000b,ln) #reset FEC_counter
        wreg(FEC_ANA_BASE_ADDR+0x1, 0x0003,ln) #release the reset 
        wreg(FEC_ANA_BASE_ADDR+0xc, err_type,ln) #set TEO error type
        wreg(FEC_ANA_BASE_ADDR+0xb, err_type,ln) #set TEi error type
    
def fec_ana_hist_DFE(lane=None):
    lanes = get_lane_list(lane)
    f0=[]
    f1=[]
    ratio=[]
    for ln in lanes: 
        #ln = lane_offset/(0x200)
        f0.append(pam4_dfe(lane=ln)[ln][0])
        f1.append(pam4_dfe(lane=ln)[ln][1])
        ratio.append(pam4_dfe(lane=ln)[ln][2])
        print 'A%d: DFE %4.2f, %4.2f, %4.2f'%(ln, f0[ln], f1[ln], ratio[ln])
        #fmt = 'A%d: DFE %4.2f, %4.2f, %4.2f'%(ln, f0[ln], f1[ln], ratio[ln])
        #file.obj.write(fmt)
        lane_hist_file_ptr = open('S%s_%s_data.txt'%(gSlice,lane_name_list[ln]),'a')
        lane_hist_file_ptr.write("%s\n%s\n"%(f0[ln],f1[ln]))
        lane_hist_file_ptr.close()
        
    return f0, f1, ratio
    
def fec_ana_hist_all(hist_time=10,lane=None, slice=None, min_bin=5, file_name='fec_ana_histogram_all.txt'):

    per_lane_data_files_en=0    # en/dis to save individual files per lanes
    pass_fail_result_print_en=1
    
    lanes = get_lane_list(lane)
    slices = get_slice_list(slice)
    
    ### FEC Analyzer Parameters
    FEC_ANA_BASE_ADDR = 0x1C0
    num_groups=4
    bins_per_group=4
    fec_ana_err_type=0
    fec_ana_T=15
    fec_ana_M=10
    fec_ana_N=5440

    ### Create empty list for Hist Data
    hist_slice_lane_bin=[] # list to contain each lane's hist counts
    hist_pass_fail_list=[] # list to conatian each lane's pass-fail status
    for slc in range(2):
        hist_slice_lane_bin.append([])
        hist_pass_fail_list.append([])
        for ln in range(16):
            hist_slice_lane_bin[slc].append([])
            hist_pass_fail_list[slc].append([])
            hist_pass_fail_list[slc][ln] = 0
            for bin in range(16):
                hist_slice_lane_bin[slc][ln].append([])
            
    ### Header for Histogram Data
    timestr = time.strftime("%Y%m%d_%H%M%S")
    fmt=('\n\n...FEC Analyzer Histogram, %d sec per bin, Slice%s '%(hist_time,'s' if len(slices)>1 else ''))
    fmt+= str(slices)
    fmt+= ', TimeStamp [%s]'%timestr
    print fmt,
    hist_file_ptr = open(file_name, 'a')
    hist_file_ptr.write(fmt)
    hist_file_ptr.close()
    
    ### Start Histogram Collection for all target slices/lanes
    print("\n...Histogram Collection in Progress..."),
    for bin_grp in range(num_groups):
        print("BinGrp%d["%(bin_grp)),
        
        ### Initialize FEC Analyzers for all target slices/lanes
        for slc in slices:
            sel_slice(slc)       
            print("S%d%s"%(slc,' ,' if slc==0 else ' ] ...')),        
            #fec_ana_hist_DFE(lanes)    
            #fec_ana_hist_one_bin_grp_setup(bin_grp,fec_ana_err_type, fec_ana_T,fec_ana_M,fec_ana_N,lanes) # setup the FEC
            for ln in lanes:
                wreg([FEC_ANA_BASE_ADDR+0x5,[12]],1,ln)     #reset Histogram
                wreg([FEC_ANA_BASE_ADDR+0x5,[12]],0,ln)     #reset Histogram
                wreg([FEC_ANA_BASE_ADDR+0x5,[2,0]], bin_grp, ln)  #select which group of histogram to read
                wreg( FEC_ANA_BASE_ADDR+0x9, 0x6202,ln) #set PRBS mode: force reload
                wreg( FEC_ANA_BASE_ADDR+0x8, (rreg(FEC_ANA_BASE_ADDR+0x8,ln) | 0x0018),ln) #set PRBS mode:PRBS31
                wreg( FEC_ANA_BASE_ADDR+0x0, ((rreg(FEC_ANA_BASE_ADDR+0x0,ln) & 0xFFF0) | fec_ana_M),ln)  #set M = 10
                wreg( FEC_ANA_BASE_ADDR+0x4, fec_ana_N,ln) #set N = 5440
                wreg( FEC_ANA_BASE_ADDR+0x5, (rreg(FEC_ANA_BASE_ADDR+0x5,ln) & 0xf007) | ((fec_ana_T<<7)+(fec_ana_T<<3)) ,ln) #set T = 2
                wreg( FEC_ANA_BASE_ADDR+0x1, 0x000b,ln) #reset FEC_counter
                wreg( FEC_ANA_BASE_ADDR+0x1, 0x0003,ln) #release the reset 
                wreg( FEC_ANA_BASE_ADDR+0xc, fec_ana_err_type,ln) #set TEO error type
                wreg( FEC_ANA_BASE_ADDR+0xb, fec_ana_err_type,ln) #set TEi error type
            
            
        ### Wait Time for Histogram Data Collection
        time.sleep(hist_time)

        ### Capture FEC Analyzers Histogram for all target slices/lanes
        for slc in slices:
            sel_slice(slc)       
            for ln in lanes:
                for bin in range(bins_per_group):
                    wreg(FEC_ANA_BASE_ADDR+0xD, 12+bin*2 ,ln)   # set reading data of histogram 0 lower 16 bit
                    histo_lo = rreg(FEC_ANA_BASE_ADDR+0x7,ln)   # read data
                    wreg(FEC_ANA_BASE_ADDR+0xD, 13+bin*2 ,ln)   # set reading data of histogram 0 upper 16 bit
                    histo_hi = rreg(FEC_ANA_BASE_ADDR+0x7,ln)   # read data
                    hist_cnt = (histo_hi*65536+histo_lo)
                    hist_slice_lane_bin[slc][ln][bin_grp*bins_per_group+bin]=hist_cnt   # get the 32-bit data                   
                    if (bin_grp*bins_per_group+bin) >= min_bin and hist_cnt > 0:
                        hist_pass_fail_list[slc][ln] = 1
    
    ### Finished Histogram Collection for all 16 bins. Print/save results
    for slc in slices:
        sel_slice(slc)
        
        #### Print Histogram Data for each slice
        fmt=("\n\nBin")
        for ln in lanes:
            fmt+=("   S%d_%s  " %(slc,lane_name_list[ln]))
        fmt+=("\n---")
        for ln in lanes:
            fmt+=(" ---------")
        for bin_grp in range(num_groups):
            for bin in range(bins_per_group):
                fmt+= '\n%-3d' %(bin_grp*bins_per_group+bin)
                for ln in lanes:
                    cnt = hist_slice_lane_bin[slc][ln][bin_grp*bins_per_group+bin]
                    fmt+= " %-9s" %(str(cnt) if cnt!=0 else '.')                
        ##### Print Pass/Fail Results per lane
        fmt+="\n   "
        for ln in lanes:
            fmt+=("%-10s" %('' if hist_pass_fail_list[slc][ln] == 0 else ' **FAIL**'))
        print fmt,
        
        #### Save Histogram Data for both slices in the 'fec_ana_hist_all.txt' file
        hist_file_ptr = open(file_name,'a')
        hist_file_ptr.write(fmt)
        hist_file_ptr.close()
        
        #### Save Histogram Data per-slice per-lane in individual files (if enabled)
        if per_lane_data_files_en:
            for ln in lanes:
                fmt=('\n...FEC Analyzer Histogram Slice %d Lane %s, %d sec per bin, TimeStamp [%s]\n'%(slc,lane_name_list[ln],hist_time,timestr))
                for bin_grp in range(num_groups):
                    for bin in range(bins_per_group):
                        fmt+=("%d\n"%hist_slice_lane_bin[slc][ln][bin_grp*bins_per_group+bin])
                lane_hist_file_ptr = open('fec_ana_hist_S%s_%s.txt'%(gSlice,lane_name_list[ln]),'a')
                lane_hist_file_ptr.write(fmt)
                lane_hist_file_ptr.close()

    return (sum([i.count(1) for i in hist_pass_fail_list]))
def fec_ana_hist_init(hist_time=10,lane=None, slice=None, min_bin=5, file_name='fec_ana_histogram_all.txt'):

    per_lane_data_files_en=0    # en/dis to save individual files per lanes
    pass_fail_result_print_en=1
    
    lanes = get_lane_list(lane)
    slices = get_slice_list(slice)
    
    ### FEC Analyzer Parameters
    FEC_ANA_BASE_ADDR = 0x1C0
    num_groups=4
    bins_per_group=4
    fec_ana_err_type=0
    fec_ana_T=15
    fec_ana_M=10
    fec_ana_N=5440
     
    ### Start Histogram Collection for all target slices/lanes
    print("\n...Histogram Initialization in Progress..."),
    for bin_grp in range(num_groups):
        print("BinGrp%d["%(bin_grp)),
        
        ### Initialize FEC Analyzers for all target slices/lanes
        for slc in slices:
            sel_slice(slc)       
            print("S%d%s"%(slc,' ,' if slc==0 else ' ] ...')),        
            #fec_ana_hist_DFE(lanes)    
            #fec_ana_hist_one_bin_grp_setup(bin_grp,fec_ana_err_type, fec_ana_T,fec_ana_M,fec_ana_N,lanes) # setup the FEC
            for ln in lanes:
                wreg([FEC_ANA_BASE_ADDR+0x5,[12]],1,ln)     #reset Histogram
                wreg([FEC_ANA_BASE_ADDR+0x5,[12]],0,ln)     #reset Histogram
                wreg([FEC_ANA_BASE_ADDR+0x5,[2,0]], bin_grp, ln)  #select which group of histogram to read
                wreg( FEC_ANA_BASE_ADDR+0x9, 0x6202,ln) #set PRBS mode: force reload
                wreg( FEC_ANA_BASE_ADDR+0x8, (rreg(FEC_ANA_BASE_ADDR+0x8,ln) | 0x0018),ln) #set PRBS mode:PRBS31
                wreg( FEC_ANA_BASE_ADDR+0x0, ((rreg(FEC_ANA_BASE_ADDR+0x0,ln) & 0xFFF0) | fec_ana_M),ln)  #set M = 10
                wreg( FEC_ANA_BASE_ADDR+0x4, fec_ana_N,ln) #set N = 5440
                wreg( FEC_ANA_BASE_ADDR+0x5, (rreg(FEC_ANA_BASE_ADDR+0x5,ln) & 0xf007) | ((fec_ana_T<<7)+(fec_ana_T<<3)) ,ln) #set T = 2
                wreg( FEC_ANA_BASE_ADDR+0x1, 0x000b,ln) #reset FEC_counter
                wreg( FEC_ANA_BASE_ADDR+0x1, 0x0003,ln) #release the reset 
                wreg( FEC_ANA_BASE_ADDR+0xc, fec_ana_err_type,ln) #set TEO error type
                wreg( FEC_ANA_BASE_ADDR+0xb, fec_ana_err_type,ln) #set TEi error type

def fec_ana_hist_collect(hist_time=10,lane=None, slice=None, min_bin=5, file_name='fec_ana_histogram_all.txt'):

    per_lane_data_files_en=0    # en/dis to save individual files per lanes
    pass_fail_result_print_en=1
    
    lanes = get_lane_list(lane)
    slices = get_slice_list(slice)
    
    ### FEC Analyzer Parameters
    FEC_ANA_BASE_ADDR = 0x1C0
    num_groups=4
    bins_per_group=4
    fec_ana_err_type=0
    fec_ana_T=15
    fec_ana_M=10
    fec_ana_N=5440

    ### Create empty list for Hist Data
    hist_slice_lane_bin=[] # list to contain each lane's hist counts
    hist_pass_fail_list=[] # list to conatian each lane's pass-fail status
    for slc in range(2):
        hist_slice_lane_bin.append([])
        hist_pass_fail_list.append([])
        for ln in range(16):
            hist_slice_lane_bin[slc].append([])
            hist_pass_fail_list[slc].append([])
            hist_pass_fail_list[slc][ln] = 0
            for bin in range(16):
                hist_slice_lane_bin[slc][ln].append([])
            
    
    ### Start Histogram Collection for all target slices/lanes
    print("\n...Histogram Collection in Progress..."),
    for bin_grp in range(num_groups):
        print("BinGrp%d["%(bin_grp)),
        
        ### Initialize FEC Analyzers for all target slices/lanes
        for slc in slices:
            sel_slice(slc)       
            print("S%d%s"%(slc,' ,' if slc==0 else ' ] ...')),        
            #fec_ana_hist_DFE(lanes)    
            #fec_ana_hist_one_bin_grp_setup(bin_grp,fec_ana_err_type, fec_ana_T,fec_ana_M,fec_ana_N,lanes) # setup the FEC
            for ln in lanes:
                wreg([FEC_ANA_BASE_ADDR+0x5,[12]],1,ln)     #reset Histogram
                wreg([FEC_ANA_BASE_ADDR+0x5,[12]],0,ln)     #reset Histogram
                wreg([FEC_ANA_BASE_ADDR+0x5,[2,0]], bin_grp, ln)  #select which group of histogram to read
                wreg( FEC_ANA_BASE_ADDR+0x9, 0x6202,ln) #set PRBS mode: force reload
                wreg( FEC_ANA_BASE_ADDR+0x8, (rreg(FEC_ANA_BASE_ADDR+0x8,ln) | 0x0018),ln) #set PRBS mode:PRBS31
                wreg( FEC_ANA_BASE_ADDR+0x0, ((rreg(FEC_ANA_BASE_ADDR+0x0,ln) & 0xFFF0) | fec_ana_M),ln)  #set M = 10
                wreg( FEC_ANA_BASE_ADDR+0x4, fec_ana_N,ln) #set N = 5440
                wreg( FEC_ANA_BASE_ADDR+0x5, (rreg(FEC_ANA_BASE_ADDR+0x5,ln) & 0xf007) | ((fec_ana_T<<7)+(fec_ana_T<<3)) ,ln) #set T = 2
                wreg( FEC_ANA_BASE_ADDR+0x1, 0x000b,ln) #reset FEC_counter
                wreg( FEC_ANA_BASE_ADDR+0x1, 0x0003,ln) #release the reset 
                wreg( FEC_ANA_BASE_ADDR+0xc, fec_ana_err_type,ln) #set TEO error type
                wreg( FEC_ANA_BASE_ADDR+0xb, fec_ana_err_type,ln) #set TEi error type
            
            
        ### Wait Time for Histogram Data Collection
        time.sleep(hist_time)

        ### Capture FEC Analyzers Histogram for all target slices/lanes
        for slc in slices:
            sel_slice(slc)       
            for ln in lanes:
                for bin in range(bins_per_group):
                    wreg(FEC_ANA_BASE_ADDR+0xD, 12+bin*2 ,ln)   # set reading data of histogram 0 lower 16 bit
                    histo_lo = rreg(FEC_ANA_BASE_ADDR+0x7,ln)   # read data
                    wreg(FEC_ANA_BASE_ADDR+0xD, 13+bin*2 ,ln)   # set reading data of histogram 0 upper 16 bit
                    histo_hi = rreg(FEC_ANA_BASE_ADDR+0x7,ln)   # read data
                    hist_cnt = (histo_hi*65536+histo_lo)
                    hist_slice_lane_bin[slc][ln][bin_grp*bins_per_group+bin]=hist_cnt   # get the 32-bit data                   
                    if (bin_grp*bins_per_group+bin) >= min_bin and hist_cnt > 0:
                        hist_pass_fail_list[slc][ln] = 1
    
    ### Finished Histogram Collection for all 16 bins. Print/save results
    for slc in slices:
        sel_slice(slc)
        
        #### Print Histogram Data for each slice
        fmt=("\n\nBin")
        for ln in lanes:
            fmt+=("   S%d_%s  " %(slc,lane_name_list[ln]))
        fmt+=("\n---")
        for ln in lanes:
            fmt+=(" ---------")
        for bin_grp in range(num_groups):
            for bin in range(bins_per_group):
                fmt+= '\n%-3d' %(bin_grp*bins_per_group+bin)
                for ln in lanes:
                    cnt = hist_slice_lane_bin[slc][ln][bin_grp*bins_per_group+bin]
                    fmt+= " %-9s" %(str(cnt) if cnt!=0 else '.')                
        ##### Print Pass/Fail Results per lane
        fmt+="\n   "
        for ln in lanes:
            fmt+=("%-10s" %('' if hist_pass_fail_list[slc][ln] == 0 else ' **FAIL**'))
        print fmt,
        
        #### Save Histogram Data for both slices in the 'fec_ana_hist_all.txt' file
        hist_file_ptr = open(file_name,'a')
        hist_file_ptr.write(fmt)
        hist_file_ptr.close()
        
        #### Save Histogram Data per-slice per-lane in individual files (if enabled)
        if per_lane_data_files_en:
            for ln in lanes:
                fmt=('\n...FEC Analyzer Histogram Slice %d Lane %s, %d sec per bin, TimeStamp [%s]\n'%(slc,lane_name_list[ln],hist_time,timestr))
                for bin_grp in range(num_groups):
                    for bin in range(bins_per_group):
                        fmt+=("%d\n"%hist_slice_lane_bin[slc][ln][bin_grp*bins_per_group+bin])
                lane_hist_file_ptr = open('fec_ana_hist_S%s_%s.txt'%(gSlice,lane_name_list[ln]),'a')
                lane_hist_file_ptr.write(fmt)
                lane_hist_file_ptr.close()

    return (sum([i.count(1) for i in hist_pass_fail_list]))
####################################################################################################
#
#
####################################################################################################
def linkTrainingSetup( self, duration=10 ): # <<<
    txPllClkSrc = 'RefClk'
    print "!!***%s: txPllClkSrc = %s" % ( fullName, txPllClkSrc )
    ltBaseAddr = 0x3000 + ( 0x100 * sdkOffset )
    # Disable link parner's LT and LT-timeout
    #wreg( ltBaseAddr + 0x00, 0x0003) # reset, enable
    #wreg( ltBaseAddr + 0x00, 0x0000) # turn off
    time.sleep( 1 )
    wreg( ltBaseAddr + 0x33, 0x0139) # allow m2 go positive
    #wreg( ltBaseAddr + 0x38, 0x94a2) # request registers: PRESET2
    wreg( ltBaseAddr + 0x38, 0x94a1) # request registers: PRESET1
    wreg( ltBaseAddr + 0x39, 0x9672) # 9672(m2 inc) or 9682(m2 dec)
    wreg( ltBaseAddr + 0x3a, 0x0003)
    wreg( ltBaseAddr + 0x00, 0x0003) # reset, enable
    wreg( ltBaseAddr + 0x00, 0x0000) # turn off
    wreg( ltBaseAddr + 0x0a, 0x0b20) # disable max_wait_timer
    wreg( ltBaseAddr + 0x08, 0xffff) # max wait_timer MSB
    #txParams.mainCursor = 22
    #txParams.pre2Cursor = 0
    #txParams.preCursor = -3
    #txParams.postCursor = -2
    # total max 31
    #### PRESET settings
    #### PRESET1: m2/m1/main/p1: 0/0/27/0
    #### PRESET2: m2/m1/main/p1: 0/-4/20/-3
    #### PRESET3: m2/m1/main/p1: 0/-7/20/0
    # All presets are 2's complement
    wreg( ltBaseAddr + 0x2a, 0x1f00) #  limit max[14:8] main 31, min[5:0] 0
    wreg( ltBaseAddr + 0x34, 0x0000) # pre2, preset1 and preset2 values
    wreg( ltBaseAddr + 0x35, 0x0000) # pre2, preset3
    wreg( ltBaseAddr + 0x31, 0x003c) # pre, preset1 and preset2 
    wreg( ltBaseAddr + 0x32, 0x3900) # pre, preset3 
    wreg( ltBaseAddr + 0x2b, 0x1b14) # main, preset 1 and preset2
    wreg( ltBaseAddr + 0x2c, 0x1400) # main, preset3
    wreg( ltBaseAddr + 0x2e, 0x003d) # post1, preset1 and preset2
    wreg( ltBaseAddr + 0x2f, 0x0000) # post1, preset3
    #wreg( ltBaseAddr + 0x00, 0x0003) # reset link training
    #wreg( ltBaseAddr + 0x45, 0x9440) # block ic_status out
    wreg( 0xa0 , 0x8300) # use SERDES tx taps, not LT
    #wreg( 0xa0, 0x8310) # use LT tap values, after ltFrameLocked bug is gone

    # Enable link parner's LT here
    wreg( ltBaseAddr + 0x00, 0x0002) # turn on link training
    dumpLtStuffForDuration(duration)

####################################################################################################
def dumpLtStuffForDuration( self, duration=1 ): # <<<
    endTime = time.time() + duration
    localTrainedTimeStamp = None
    n = neighbor
    while time.time() < endTime:
        localRx = rreg( 0x3000 + ( 0x100 * sdkOffset ) + 4 )
        remoteRx = neighbor.ham.read16(0x97)
        if (localRx & 0x800) and (localTrainedTimeStamp == None):
            print "***Switched to LT taps"
            wreg( 0xa0, 0x8310) # use LT tap values
            localTrainedTimeStamp = time.time()
        print "local @0x4=0x%x, remote @0x97=0x%x" % (localRx, remoteRx )
        if (localRx & 0x800) and (remoteRx & 1 ):
            print "BOTH sides trained !!!, time lapsed from localTrained = ", ( time.time() - localTrainedTimeStamp )
            if (n.txPreCursor != -4) or (n.txMainCursor != 156) or (n.txPostCursor != -4):
                print "show neighbor's rx/tx debug info"
            #return
            time.sleep( 0.5 )
        else:
            print "SHOW neighbor's rx/tx debug info"
####################################################################################################

def channel_tap_analyzer (ow_dac=0xf,ctle_val=7,gain_clear=1,ftap=16,lane=0):
    #define
    hf_cnt_owen_addr = 0x16
    hf_cnt_owen_bit_loc = [7]
    hf_cnt_ow_addr = 0x26
    sd_thre_addr = 0x3
    sd_owen_addr = 0x23
    sd_owen_bit_loc = [1]
    sd_ow_addr = 0x18
    sd_ow_bit_loc = [15]
    ow_dac_addr = 0x21
    ow_dac_bit_loc = [12,8]
    cnt_3_addr = 0x6f
    cnt_0_addr = 0x70 
    cnt_hf_addr = 0x72
    lane_reset_addr=0x00
    lane_reset_bit_loc=[15]
    acal_start_addr = 0x1f
    acal_start_owen_bit_loc = [1]
    acal_start_ow_bit_loc = [0]
    acal_done_addr = 0x23
    acal_done_owen_bit_loc = [3]
    acal_done_ow_bit_loc = [2]
    fast_rotater_addr = 0x22
    fast_rotater_bit_loc = [15,14]
    dir_bit_loc = [13,12]
    cth_gray_addr = 0x17
    cth_gray_bit_loc = [7,0]
    b1_addr = 0x18
    b1_bit_loc = [6,4]
    b_addr = 0x22
    b_bit_loc = [11,9]
    bn_bit_loc = [8,6]
    ths_owen_addr = 0x12
    ths_owen_bit_loc = [15]
    ths_ow_addr = 0x13
    ths_ow_bit_loc = [15,4]
    mcounter_addr = 0x31
    timer_addr = 0xa
    dac_timer_bit_loc = [9,5]
    dac_timer = 14
    ctle_timer_bit_loc = [4,0]
    ctle_timer = 14
    md_addr = 0x21
    md_ow_bit_loc = [3,2]
    mu_mg_ow_addr = 0x20
    mu_mg_ow_bit_loc = [15,13]
    post_dis_addr = 0x41
    post_dis_bit_loc = [14]

    bp_addr = 0x11
    bp1_en_bit_loc = [15]
    bp2_en_bit_loc = [14]
    bp1_bit_loc = [12,8]
    bp2_bit_loc = [4,0]
    bp_ctu_bit_loc = [13]
    step_en_bit_loc = [7]
    step_bit_loc = [6]

    iter_s6_addr = 0x09
    iter_s6_bit_loc = [3,0]
    
    iter_s4_addr = 0x06
    iter_s4_bit_loc = [15,12]
    
    timer_s_addr = 0x07
    timer_s5_bit_loc = [14,11]
    
    timer_s1_addr = 0x00
    timer_s1_bit_loc = [4,0]
    th_ini_bit_loc = [14,8]
    
    timer_s3_addr = 0x03
    timer_s3_bit_loc = [15,11]
    
    pam4_tap_sel_addr = 0x45
    pam4_tap_sel_bit_loc = [6,3]
    
    ctle_en_addr = 0x21
    ctle_en_bit_loc = [7]
    ctle_val_addr = 0x21
    ctle_val_bit_loc = [6,4]
    ctle_gain1_addr = 0xF5
    ctle_gain1_bit_loc = [11,8]
    ctle_gain2_addr = 0xF5
    ctle_gain2_bit_loc = [7,4]
    ctle_map2_addr = 0x48
    ctle_map1_addr = 0x49
    ctle_map0_addr = 0x4a
    sm_addr = 0x28
    sm_bit_loc = [13,9]
    sm_bp1_bit_loc = [15]
    sm_bp2_bit_loc = [14]

    global gF1over3Min  # <<<< ALEX
    #record current setting
    bp1_ori = rregBits(bp_addr, bp1_bit_loc,lane)
    bp1_en_ori = rregBits(bp_addr, bp1_en_bit_loc,lane)
    bp2_en_ori = rregBits(bp_addr, bp2_en_bit_loc,lane)
    ctle_map_ori = rregBits(ctle_map0_addr, [5,0],lane)
    ctle_ori = rregBits(ctle_val_addr, ctle_val_bit_loc,lane)
    ts5_ori = rregBits(timer_s_addr, timer_s5_bit_loc,lane)
    iter4_ori = rregBits(iter_s4_addr, iter_s4_bit_loc,lane)
    ts1_ori = rregBits(timer_s1_addr, timer_s1_bit_loc,lane)
    ts3_ori = rregBits(timer_s3_addr, timer_s3_bit_loc,lane)
    mu_ori = rregBits(mu_mg_ow_addr, mu_mg_ow_bit_loc,lane)
    f1_ori = f13(val = None, lane = lane)#[lane]
    
    #initial setting
    start_time=time.time()
    wregBits(pam4_tap_sel_addr, pam4_tap_sel_bit_loc, ftap,lane)
    wregBits(fast_rotater_addr, fast_rotater_bit_loc, 3,lane)
    wregBits(fast_rotater_addr, dir_bit_loc, 2,lane)
    wregBits(cth_gray_addr, cth_gray_bit_loc, 0x80,lane)

    wregBits(timer_s_addr, timer_s5_bit_loc, 0x9,lane)
    wregBits(timer_s1_addr, timer_s1_bit_loc, 0x1,lane)
    wregBits(timer_s3_addr, timer_s3_bit_loc, 0x1,lane)
    wregBits(iter_s4_addr, iter_s4_bit_loc, 0x1,lane)
    wregBits(mu_mg_ow_addr, mu_mg_ow_bit_loc, 0x4,lane)
    wregBits(timer_addr, dac_timer_bit_loc, 1,lane)
    wregBits(timer_addr, ctle_timer_bit_loc, 1,lane)
    wreg(hf_cnt_ow_addr, 0x8000,lane)
    wregBits(hf_cnt_owen_addr, hf_cnt_owen_bit_loc, 1,lane)
    wregBits(ow_dac_addr, ow_dac_bit_loc, (0x10+ow_dac),lane)
    dcgain = dc_gain(lane=lane)
    
    if (ctle_val==8):
        wregBits(ctle_map0_addr,[5,0],0x3f,lane)
        ctle_val = 7
    wregBits(ctle_val_addr, ctle_val_bit_loc, ctle_val,lane)
    if(gDebugTuning):print "\n\nChannel Tap Analyzer: DAC: %2d" % (rregBits(ow_dac_addr, ow_dac_bit_loc)&0xF,lane),
    if(gDebugTuning):print "CTLE: %2d" % rregBits(ctle_val_addr, ctle_val_bit_loc,lane),
    
    if ftap<16:
        wregBits(md_addr, md_ow_bit_loc, 0x2,lane)
        wregBits(post_dis_addr, post_dis_bit_loc, 0x1,lane)
    else:
        f13(0,lane)
        #f13(gF1over3Min)

    #main loop
    hfList = []
    for i in [0,1]:
        wregBits(b_addr, b_bit_loc, 0x7,lane)

        if i==1:
            wregBits(b1_addr, b1_bit_loc, 0x7,lane)
            wregBits(b_addr, bn_bit_loc, 0x6,lane)
        else:
            wregBits(b1_addr, b1_bit_loc, 0x4,lane) #FC: I think it should be 0x7, it used to be 0x4, Alex, can you take a try? 
            wregBits(b_addr, bn_bit_loc, 0x5,lane)

        
        if gain_clear:
            #wregBits(ctle_gain1_addr, ctle_gain1_bit_loc, 0,lane)
            dc_gain(agcgain1=0,lane = lane)#[lane]
        if(gDebugTuning):
            ctle_gain1_val, ffe_gain1_val, ctle_gain2_val, ffe_gain2_val = dc_gain()
            print "\nChannel Tap Analyzer: CTLE_Gain1:%2d,FFE_Gain1:%2d,CTLE_Gain2:%2d,FFE_Gain2:%2d" % (ctle_gain1_val, ffe_gain1_val, ctle_gain2_val, ffe_gain2_val),
        #if(gDebugTuning):print "CTLE_Gain1: %2d" % rregBits(ctle_gain1_addr, ctle_gain1_bit_loc),
        
        
        hf_thre = 0x1f
        final_cnt = 0
        final_hf = 0x0
        wregBits(sd_owen_addr, sd_owen_bit_loc, 1,lane)
        wregBits(sd_ow_addr, sd_ow_bit_loc, 1,lane)
        for t in range(1,9):
            wregBits(lane_reset_addr, lane_reset_bit_loc, 1,lane)
            wregBits(acal_start_addr, acal_start_owen_bit_loc, 1,lane)
            wregBits(acal_start_addr, acal_start_ow_bit_loc, 0,lane)
            wregBits(acal_done_addr, acal_done_owen_bit_loc, 1,lane)
            wregBits(acal_done_addr, acal_done_ow_bit_loc, 0,lane)
            wregBits(bp_addr, bp1_en_bit_loc, 0,lane)
            wregBits(bp_addr, bp2_en_bit_loc, 0,lane)
            wregBits(bp_addr, bp1_bit_loc, 0x11,lane)
            sm_cont(lane=lane)
            wregBits(bp_addr, bp1_en_bit_loc, 1,lane)
            wregBits(lane_reset_addr, lane_reset_bit_loc, 0,lane)
            wregBits(timer_s1_addr, th_ini_bit_loc, hf_thre,lane)
            wregBits(acal_done_addr, acal_done_owen_bit_loc, 0,lane)
            wregBits(acal_start_addr, acal_start_owen_bit_loc, 0,lane)
            wait_for_bp1_timeout=0
            while True:
                bp1=rregBits(sm_addr,sm_bp1_bit_loc,lane)
                if bp1:
                        #print hex(rregBits(pam4_dfe_read_addr, pam4_dfe_read_bit_loc))
                    step =  (t<7) and (0x40>>(t+1)) or 1

                    if rreg(mcounter_addr,lane)>0x10:
                        #if(gDebugTuning):print ("\n Channel Tap Analyzer: Hit: hf_thre: %2d, st: %2d, cnt_hf: %2d" % (hf_thre, rregBits(sm_addr, sm_bit_loc), rregLane(mcounter_addr)))
                        final_hf = hf_thre
                        if hf_thre<0x3f:
                            final_cnt = rreg(mcounter_addr,lane)
                            hf_thre =min(hf_thre+step, 0x3f) 
                    else:
                        #if(gDebugTuning):print ("\n Channel Tap Analyzer:Miss: hf_thre: %2d, st: %2d, cnt_hf: %2d" % (hf_thre, rregBits(sm_addr, sm_bit_loc), rregLane(mcounter_addr)))
                        hf_thre = hf_thre - step
                    break
                else:
                    wait_for_bp1_timeout+=1
                    if wait_for_bp1_timeout>1000:
                        if(gDebugTuning):print ("\n Channel Tap Analyzer: ***>> Timed out waiting for BP1")
                        #save_setup('56G_18in_smart_reset_bp1_timeout_reg_dump.txt')
                        break
                        
        hfList.append(final_hf)
    f0 = (hfList[1]+hfList[0])/2
    f1 = (hfList[1]-hfList[0])/3
    if f0 ==0:
        f0=0.0000001
    tap_value = int(round(f1*50/f0))
    print "\nChannel Tap Analyzer: f1/f0: %1.3f/%1.3f = %2.3f,  tap iter1: %2d,  tap iter0: %2d" % (f1,f0,tap_value, hfList[1], hfList[0])
    if(gDebugTuning):print "\nChannel Tap Analyzer: f1/f0: %1.3f/%1.3f = %2.3f,  tap iter1: %2d,  tap iter0: %2d" % (f1,f0,tap_value, hfList[1], hfList[0]),
    if(gDebugTuning):print "\nChannel Tap Analyzer: f1o3 Tap: %2.0f" % tap_value,

    

    f13(f1_ori[0],lane)
    dc_gain(dcgain[lane][0],dcgain[lane][1],dcgain[lane][2],dcgain[lane][3],lane=lane)
    wregBits(post_dis_addr, post_dis_bit_loc, 0x0,lane)
    wregBits(md_addr, md_ow_bit_loc, 0x0,lane)
    wregBits(sd_owen_addr, sd_owen_bit_loc, 0,lane)
    wregBits(fast_rotater_addr, fast_rotater_bit_loc, 0,lane)
    wregBits(fast_rotater_addr, dir_bit_loc, 0,lane)
    wregBits(cth_gray_addr, cth_gray_bit_loc, 0x0,lane)
    wregBits(ths_owen_addr, ths_owen_bit_loc, 0,lane)
    wregBits(b1_addr, b1_bit_loc, 0,lane)
    wregBits(b_addr, b_bit_loc, 0,lane)
    wregBits(b_addr, bn_bit_loc, 0,lane)
    wregBits(hf_cnt_owen_addr, hf_cnt_owen_bit_loc, 0,lane)
    #wregBits(ctle_val_addr, ctle_val_bit_loc, ctle_ori)
    wregBits(timer_s_addr, timer_s5_bit_loc, ts5_ori,lane)
    wregBits(timer_s1_addr, timer_s1_bit_loc, ts1_ori,lane)
    wregBits(timer_s3_addr, timer_s3_bit_loc, ts3_ori,lane)
    wregBits(mu_mg_ow_addr, mu_mg_ow_bit_loc, mu_ori,lane)
    wregBits(ctle_map0_addr,[5,0],ctle_map_ori,lane)
    wregBits(ow_dac_addr, ow_dac_bit_loc, 0,lane)
    wregBits(timer_addr, dac_timer_bit_loc, 10,lane)
    wregBits(timer_addr, ctle_timer_bit_loc, 10,lane)
    wregBits(iter_s4_addr, iter_s4_bit_loc, iter4_ori,lane)
    wregBits(bp_addr, bp1_en_bit_loc, 0,lane)
    wregBits(bp_addr, bp2_en_bit_loc, 0,lane)
    wregBits(bp_addr, bp1_bit_loc, bp1_ori,lane)
    wregBits(bp_addr, bp1_en_bit_loc, bp1_en_ori,lane)
    wregBits(bp_addr, bp2_en_bit_loc, bp2_en_ori,lane)
    sm_cont(lane=lane)
    lr(lane)
    if(gDebugTuning):print ('\nChannel Tap Analyzer: OptTime: %2.2f, '%(time.time()-start_time))
#    print tap_value
    return tap_value


def pu_intp_nrz(lane):
    wreg([0x13b,[10,8]], 0x4,lane) # OWEN_Ktheta = 1, Ktheta=0
    wreg([0x163,[15,8]],0x80,lane) # OWEN Phase=1  Phase1=0x00
    wreg([0x163, [7,0]],0xe0,lane) # Phase2_flip=1 Phase2=0x60
    wreg([0x164,[15,8]],0xe0,lane) # Phase3_flip=1 Phase3=0x60
    wreg([0x164, [7,0]],0xe0,lane) # Phase4_flip=1 Phase4=0x60
    wreg([0x161, [7,0]],0x80,lane) # OWEN Theta2=1 Theta2=0x00
    wreg([0x162,[15,8]],0x80,lane) # OWEN Theta3=1 Theta3=0x00
    wreg([0x162, [7,0]],0x80,lane) # OWEN Theta4=1 Theta4=0x00
    time.sleep(0.100)
    wreg([0x163,[15,8]],0x00,lane) # OWEN Phase=0  Phase1=0x00
    wreg([0x163, [7,0]],0x80,lane) # Phase2_flip=1 Phase2=0x00
    wreg([0x164,[15,8]],0x80,lane) # Phase3_flip=1 Phase3=0x00
    wreg([0x164, [7,0]],0x80,lane) # Phase4_flip=1 Phase4=0x00
    wreg([0x161, [7,0]],0x00,lane) # OWEN Theta2=0 Theta2=0x00
    wreg([0x162,[15,8]],0x00,lane) # OWEN Theta3=0 Theta3=0x00
    wreg([0x162, [7,0]],0x00,lane) # OWEN Theta4=0 Theta4=0x00
    wreg([0x13b,[10,8]], 0x0,lane) # OWEN_Ktheta = 0, Ktheta=0
    lr(lane)

def debug_nrz_ca(lane=None) : 
    bp_reached = {}
    of_cnt     = {}
    step       = {}
    of_thre    = {}

    for x in range(0, 8) :   
        wreg(0x9807, (16)+x)
        wreg(0x9806, 0xb010+lane)
        time.sleep(.1)
        bp_reached[x] = rreg(0x9807)
    for x in range(0, 8) :   
        wreg(0x9807, (32)+x)
        wreg(0x9806, 0xb010+lane)
        time.sleep(.1)
        of_cnt[x] = rreg(0x9807)
    for x in range(0, 8) :   
        wreg(0x9807, (48)+x)
        wreg(0x9806, 0xb010+lane)
        time.sleep(.1)
        step[x] = rreg(0x9807)
    for x in range(0, 8) :   
        wreg(0x9807, (64)+x)
        wreg(0x9806, 0xb010+lane)
        time.sleep(.1)
        of_thre[x] = rreg(0x9807)
         
    
    for x in range(0, 8) :
        print('state : %d,  of_cnt : %d, step : %d, of_thre : %d' % (bp_reached[x], of_cnt[x], step[x], of_thre[x]))

def test_state_loop(lane=None) :
    c = NrzReg
    wreg(c.rx_theta_update_mode_addr, 0, lane)
    wreg(0xed, 0x8888, lane)
    delta(0, lane)
    wreg(c.rx_of_cntr_upper_limit_addr, 0xffff, lane)
    wreg(c.rx_of_cntr_lower_limit_addr, 0x0, lane)
    wreg(c.rx_hf_cntr_target_addr, 0xffff, lane)
    wreg(c.rx_of_period_addr, 14, lane)
    wreg(c.rx_hf_period_addr, 14, lane)
    wreg(c.rx_dac_ow_addr, 7, lane)
    wreg(c.rx_dac_owen_addr, 0x1, lane)
    wreg(c.rx_of_cntr_lower_limit_addr, 0xffff, lane)
    bp1(1,8,lane)
    bp2(1,9,lane) 
    wreg(c.rx_lane_rst_addr, 0, lane)
    wreg(c.rx_lane_rst_addr, 1, lane)
    wreg(c.rx_lane_rst_addr, 0, lane)
    sm_cont(lane)
    time.sleep(2)
    cnt = 0
    for t in range(1,30):   
        time.sleep(.5)
        bp1_reached=bp1(lane=lane)[lane][-1]
        bp2_reached=bp2(lane=lane)[lane][-1]
        if bp1_reached or bp2_reached :
            cnt = cnt+1
        sm_cont(lane)
        bp1(1,0xb,lane)
        bp2(1,0xc,lane)
        wreg(c.rx_of_cntr_lower_limit_addr, 0x0, lane)
        sm_cont(lane)
        time.sleep(.5)
        bp1_reached=bp1(lane=lane)[lane][-1]
        bp2_reached=bp2(lane=lane)[lane][-1]
        if bp2_reached : break
        bp1(1,8,lane)
        bp2(1,9,lane)
        wreg(c.rx_of_cntr_lower_limit_addr, 0xffff, lane)
        sm_cont(lane)        
    print "cnt number is %d" % cnt

#################################################################################################### Added by Jeff
CMD=0x9806
CMD_DETAIL=0x9807
REG_DATA=0x9f00
    
def MdioRd(addr):
    #print "MdioRd addr 0x%x " % (addr) 
    MdioRdLock.acquire()
    try:
        libcameo.mdio_read.restype=c_ushort   
        val=libcameo.mdio_read(gSlice,0x1,addr)
    finally:        
        MdioRdLock.release()    

    #print "MdioRd val 0x%x " % (val)
    return val
	#chip.MdioRd(addr)

def MdioWr(addr, value):
    #print "MdioWr addr 0x%x value 0x%x" % (addr,value)
    MdioWrLock.acquire()
    try:
        libcameo.mdio_write(gSlice,0x1,addr, value)
    finally:        
        MdioWrLock.release()           
    #chip.MdioWr(addr, value)
	

def MdioRdh(addr):
    return "0x%04x" % MdioRd(addr)
    
#################################################################################################### Added by Jeff
def BE_debug(lane, mode, index):
    MdioWr(CMD_DETAIL, index)
    cmd = 0xB000 + ((mode&0xf)<<4) + lane
    result = fw_cmd(cmd)
    if (result!=0x0b00+mode):
        raise IOError("Debug command failed with code %04x" % result)
        #print "Debug command failed with code %04x" % result
    return MdioRd(CMD_DETAIL)    
def BE_debug_signed(lane, mode, index):
    value = BE_debug(lane, mode, index)
    return value if value<0x8000 else value-0x10000
def BE_debug1(lane, mode, index, length):
    return BE_debugs(lane, mode, range(index, index+length))
def BE_debug1_signed(lane, mode, index, length):
    return BE_debugs_signed(lane, mode, range(index, index+length))
def BE_debug2(lane, mode, index, l1, l2):
    return [BE_debugs(lane, mode, range(index+i*l2, index+(i+1)*l2))
        for i in range(l1)]
def BE_debug2_signed(lane, mode, index, l1, l2):
    return [BE_debugs_signed(lane, mode, range(index+i*l2, index+(i+1)*l2))
        for i in range(l1)]
def BE_debugs(lane, mode, index):
    return [BE_debug(lane, mode, i) for i in index]
def BE_debugs_signed(lane, mode, index):
    return [BE_debug_signed(lane, mode, i) for i in index]
def BE_debug32(lane, mode, index):
    h = BE_debug(lane, mode, index)
    l = BE_debug(lane, mode, index+1)
    return (h<<16)+l
def BE_debug32_signed(lane, mode, index):
    value = BE_debug32(lane, mode, index)
    return value if value<0x80000000 else value-0x100000000


def BE_info(lane, mode, index, size):
    MdioWr(CMD_DETAIL, index)
    mode |= 8
    cmd = 0xB000 + ((mode&0xf)<<4) + lane
    result = fw_cmd(cmd)
    if (result!=0x0b00+mode):
        raise IOError("Info command failed with code %04x" % result)
    return [MdioRd(REG_DATA+i) for i in range(size)]

def BE_info_signed(lane, mode, index, size):
    return [x if x<0x8000 else x-0x10000 for x in BE_info(lane, mode, index, size)]   

def pam4_info_fw(lane):
    ISI = BE_info_signed(lane, 10, 0, 16)
    ths = BE_info_signed(lane, 10, 1, 12)
    ffe_accu = BE_info(lane, 10, 2, 8)
    ffe_accu = [(ffe_accu[i]<<16) + ffe_accu[i+1] for i in range(0, 8, 2)]
    ffe_accu = [x if x<0x80000000 else x-0x100000000 for x in ffe_accu]
    ffe_accu = [x/32768.0 for x in ffe_accu]
    ffe = BE_info_signed(lane, 10, 3, 7)
    ffe_flips = BE_debugs(lane, 2, range(660, 664))
    print "ISI =", ISI
    print "ths =", ths
    print "FFE_accu =", ffe_accu
    print "FFE: K   =", ffe[0:4], ", S = [%d %d], nbias = %d" % (ffe[5], ffe[6], ffe[4])
    print "FFE: K Pol flips=", ffe_flips[0:4]
    
def pam4_info_fw_alex(lane):
    mystring = ""
    ISI = BE_info_signed(lane, 10, 0, 16)
    ths = BE_info_signed(lane, 10, 1, 12)
    ffe_accu = BE_info(lane, 10, 2, 8)
    ffe_accu = [(ffe_accu[i]<<16) + ffe_accu[i+1] for i in range(0, 8, 2)]
    ffe_accu = [x if x<0x80000000 else x-0x100000000 for x in ffe_accu]
    ffe_accu = [x/32768.0 for x in ffe_accu]
    ffe = BE_info_signed(lane, 10, 3, 7)
    mystring += "\nISI =%s "% ISI
    mystring += "\nths = %s" % ths
    mystring += "\nFFE_accu =%s "% ffe_accu
    mystring += "\nFFE: K =%s"% ffe[0:4] 
    mystring += "\nS = [%d %d]" %(ffe[5], ffe[6])
    mystring +="\nnbias = %d" % ( ffe[4])
    return mystring
    
def pam4_show_comps(lane, f=None):
    if type(lane)==list:
        y=lane
    else:
        y = BE_info_signed(lane, 10, 1, 12)
    x=[1,1,1,2,2,2,3,3,3,4,4,4]
    if f is None:
        f=matplotlib.pyplot.figure()
    else:
        f.clf()

    l=matplotlib.pyplot.plot(x, y, 'o', figure=f)
    matplotlib.pyplot.plot(x[0::9], y[0::9], figure=f)
    matplotlib.pyplot.plot(x[1::9], y[1::9], figure=f)
    matplotlib.pyplot.plot(x[2::9], y[2::9], figure=f)
    f.show()
    return f

def dump_fw(lane):
    exit_codes = BE_debugs(lane, 0, range(100, 100+16))
    print "exit codes =", [hex(x) for x in exit_codes]
    
def nrz_dump_fw(lane):
    agcgain1_dc1 = BE_debugs(lane, 1, range(80, 80+9))
    agcgain2_dc1 = BE_debugs(lane, 1, range(100, 100+9))
    index_dc1 = BE_debugs(lane, 1, range(140, 140+9))
    of_cnt_dc1 = BE_debugs(lane, 1, range(120, 120+9))
    print "-----------------FIRST DAC SEARCH------------------"
    print "|init agcgains|index number|of_cnt|result agcgains|"
    for i in range(8) : 
        print "| (%3d,  %3d) |    %3d     |%5d |  (%3d,  %3d)  |"%(agcgain1_dc1[i], agcgain2_dc1[i], index_dc1[i], of_cnt_dc1[i], agcgain1_dc1[i+1], agcgain2_dc1[i+1])
    print "---------------------------------------------------"
    
    of_cnt_ca1 = BE_debugs(lane, 1, range(330, 330+8))
    of_thre_ca1 = BE_debugs(lane, 1, range(300, 300+8))
    hf_cnt_ca1 = BE_debugs(lane, 1, range(390, 390+8))
    hf_thre_ca1 = BE_debugs(lane, 1, range(360, 360+8))
    
    of_cnt_ca2 = BE_debugs(lane, 1, range(338, 338+8))
    of_thre_ca2 = BE_debugs(lane, 1, range(308, 308+8))
    hf_cnt_ca2 = BE_debugs(lane, 1, range(398, 398+8))
    hf_thre_ca2 = BE_debugs(lane, 1, range(368, 368+8))
    
    of_cnt_ca3 = BE_debugs(lane, 1, range(346, 346+8))
    of_thre_ca3 = BE_debugs(lane, 1, range(316, 316+8))
    hf_cnt_ca3 = BE_debugs(lane, 1, range(406, 406+8))
    hf_thre_ca3 = BE_debugs(lane, 1, range(376, 376+8))
    
    print "----------------------CHANNEL ANALYZER-----------------------"
    print "|        CA1        |        CA2        |        CA3        |"
    print "|of_cnt of hf_cnt hf|of_cnt of hf_cnt hf|of_cnt of hf_cnt hf|"
    for i in range(8) :
        print "|%5d %2d %5d %2d  |%5d %2d %5d %2d  |%5d %2d %5d %2d  |"%(of_cnt_ca1[i], of_thre_ca1[i], hf_cnt_ca1[i], hf_thre_ca1[i], of_cnt_ca2[i], of_thre_ca2[i], hf_cnt_ca2[i], hf_thre_ca2[i], of_cnt_ca3[i], of_thre_ca3[i], hf_cnt_ca3[i], hf_thre_ca3[i])
    print "-------------------------------------------------------------"
    
    ctle = BE_debugs(lane, 1, range(505, 505+4))
    dfe_ctle1 = [BE_debug_signed(lane, 1, 510+i) for i in range(3)]
    dfe_ctle2 = [BE_debug_signed(lane, 1, 513+i) for i in range(3)]
    dfe_ctle3 = [BE_debug_signed(lane, 1, 516+i) for i in range(3)]
    dfe_ctle4 = [BE_debug_signed(lane, 1, 519+i) for i in range(3)]
    print "-------CTLE FINE SEARCH--------"
    print "| ctle   |%4d|%4d|%4d|%4d|"%(ctle[0], ctle[1], ctle[2], ctle[3])
    print "| dfe F1 |%4d|%4d|%4d|%4d|"%(dfe_ctle1[0], dfe_ctle2[0], dfe_ctle3[0], dfe_ctle4[0])
    print "| dfe F2 |%4d|%4d|%4d|%4d|"%(dfe_ctle1[1], dfe_ctle2[1], dfe_ctle3[1], dfe_ctle4[1])
    print "| dfe F3 |%4d|%4d|%4d|%4d|"%(dfe_ctle1[2], dfe_ctle2[2], dfe_ctle3[2], dfe_ctle4[2])
    print "-------------------------------"  
    
    agcgain1_dc2 = BE_debugs(lane, 1, range(89, 89+9))
    agcgain2_dc2 = BE_debugs(lane, 1, range(109, 109+9))
    index_dc2 = BE_debugs(lane, 1, range(149, 149+9))
    of_cnt_dc2 = BE_debugs(lane, 1, range(129, 129+9))
    print "----------------SECOND DAC SEARCH------------------"
    print "|init agcgains|index number|of_cnt|result agcgains|"
    for i in range(8) : 
        print "| (%3d,  %3d) |    %3d     |%5d |  (%3d,  %3d)  |"%(agcgain1_dc2[i], agcgain2_dc2[i], index_dc2[i], of_cnt_dc2[i], agcgain1_dc2[i+1], agcgain2_dc2[i+1])
    print "---------------------------------------------------"
    
    debug_states = BE_debugs(lane, 1, range(160, 160+8))
    print "stats = [",
    for i in range(8) : 
        print ("%2d  "%(debug_states[i])),
    print "]"
    
def pam4_dump_fw_linear_fit(lane):
    pam4_state = BE_debug(lane, 2, 0)
    error_exit = False
    if pam4_state==0xeeef:
        error_exit = True
        pam4_state = BE_debug(lane, 0, 3)
    try:
        adapt_mode = BE_debug(lane, 2, 3)
    except:
        adapt_mode = 0
    if adapt_mode==0:
        print "Auto adapt mode"
    elif adapt_mode==1:
        print "Factory fixed setting mode"
    elif adapt_mode==2:
        print "User fixed setting mode"
    else:
        print "Unknown adapt mode"
        return

    agcgain1 = BE_debug(lane, 2, 23)
    agcgain2 = BE_debug(lane, 2, 24)
    agcgain1_dc1 = BE_debug(lane, 2, 8)
    agcgain2_dc1 = BE_debug(lane, 2, 9)
    final_of = BE_debug(lane, 2, 4)
    final_hf = BE_debug(lane, 2, 5)

    dc_search_agcgain = BE_debug2(lane, 2, 300, 2, 15)
    ratio = BE_debug(lane, 2, 2)
    agc_index = BE_debug(lane, 2, 1)
    restart_count = BE_debug(lane, 2, 7)
    delta = BE_debug_signed(lane, 2, 27)
    delta_times0 = BE_debug_signed(lane, 2, 35)+1
    delta_dump0 = [BE_debug_signed(lane, 2, 110+i) for i in range(10)]
    fm1_dump0 = [BE_debug_signed(lane, 2, 150+i) for i in range(10)]
    ctle = BE_debug(lane, 2, 33)
    next_f13 = BE_debug(lane, 2, 34)
    dc_search_agcgain[0] = [(x>>8, x&0xff) for x in dc_search_agcgain[0]]
    dc_search_agcgain[1] = [(x>>8, x&0xff) for x in dc_search_agcgain[1]]

    ctle_isi = BE_debug2_signed(lane, 2, 330, 2, 4)
    ctle_record = BE_debug1_signed(lane, 2, 340, 2)
    ctle_freq_accu =  BE_debug_signed(lane, 2, 200)
    ctle_freq_accu1 =  BE_debug_signed(lane, 2, 201)
    dump_lf=False
    if dump_lf:
        lf_dmp_size = 2
    else:
        lf_dmp_size = 7

    smart_check_result = BE_debug2_signed(lane, 2, 350, lf_dmp_size, 5)
    smart_check_ths = BE_debug2_signed(lane, 2, 700, lf_dmp_size, 12)
    #smart_check1_ths = BE_debug2_signed(lane, 2, 800, lf_dmp_size, 12)
    lf_result = BE_debug2_signed(lane, 2, 400, lf_dmp_size, 5)

    if dump_lf:
        force_ths = BE_debug2_signed(lane, 2, 900, lf_dmp_size, 12)
        plus_margin = BE_debug2_signed(lane, 2, 1000, lf_dmp_size, 12)
        minus_margin = BE_debug2_signed(lane, 2, 1100, lf_dmp_size, 12)
    em_debug = BE_debug2_signed(lane, 2, 470, lf_dmp_size, 3)
    ffe_adapt = BE_debug2_signed(lane, 2, 600, 1, 4)
    nbias_adapt = BE_debug1_signed(lane, 2, 640, 1)
    if error_exit:
        print "Error exited"
    print "PAM4 state = 0x%04x" % pam4_state
    print "restart count = %d" % restart_count
    if adapt_mode!=0:
        fixed_f13 = BE_debug_signed(lane, 2, 40)
        fixed_CTLE = BE_debug(lane, 2, 41)
        fixed_Kp = BE_debug(lane, 2, 42)
        fixed_agcgain = BE_debugs(lane, 2, range(43, 45))
        fixed_delta = BE_debug_signed(lane, 2, 45)
        fixed_skef = BE_debug(lane, 2, 46)
        fixed_edge = BE_debug(lane, 2, 47)
        fixed_kf = BE_debug(lane, 2, 48)
        fixed_sf = BE_debug(lane, 2, 49)
        ctle_temp = BE_debugs(lane, 2, range(50, 53))
        fixed_ffe = BE_debugs_signed(lane, 2, range(53, 53+7))
        # Reconstruct CTLE table
        fixed_ctle_all = (ctle_temp[0]<<32) | (ctle_temp[1]<<16) | ctle_temp[2]
        fixed_ctle_table = [(fixed_ctle_all>>(i*6))&0x3f for i in range(7, -1, -1)]
        fixed_ctle_table = [(i>>3, i&7) for i in fixed_ctle_table]
        print "Fixed settings:"
        print "    F1/3 = %d" % fixed_f13
        print "    CTLE = %d" % fixed_CTLE
        print "    Kp = %d" % fixed_Kp
        print "    agcgain = %d, %d" % (fixed_agcgain[0], fixed_agcgain[1])
        print "    delta = %d" % fixed_delta
        print "    Skef = %d (%s)" % (fixed_skef&7, "enabled" if fixed_skef&8 else "disabled")
        print "    Edge = 0x%04x" % fixed_edge
        print "    Kf = %d, Sf = %d" % (fixed_kf, fixed_sf)
        print "    CTLE table =", fixed_ctle_table
        print "    FFE: K1=%d, K2=%d, K3=%d, K4=%d, S1=%d, S1=%d, nbias=%d" % (fixed_ffe[0],
                fixed_ffe[1], fixed_ffe[2], fixed_ffe[3], fixed_ffe[4], fixed_ffe[5], fixed_ffe[6])
    if adapt_mode<1:
        print "AGCgain = %d, %d" % (agcgain1, agcgain2)
        print "=== DC search 1 ==="
        print "   agcgain record =", dc_search_agcgain[0]
        print "   AGCgain after DC search1 = %d, %d" % (agcgain1_dc1, agcgain2_dc1)
        print "=== Channel analyzer === "
        print "   OF =", final_of
        print "   HF =", final_hf
        print "   Ratio = %d (%5.3f)" % (ratio, ratio/256.0)
        print "   CTLE search index = %d" % (agc_index)
        print "=== DC search 2 === "
        print "   agcgain record =", dc_search_agcgain[1]
        print "=== Smart check and linear fit ==="
        print "   smart_chk=", smart_check_result
        print "   smart_check_ths=", smart_check_ths
        #print "   smart_check1_ths=", smart_check1_ths
        print "   LF_result =", lf_result
        if dump_lf:
            print "   force_ths=", force_ths
            print "   plus_margin=", plus_margin
            print "   minus_margin=", minus_margin
        print "   EM_debug =", em_debug
        print "   Final F1/3 init = %d" % next_f13
        print "=== CTLE search ==="
        print "   CTLE before search =", ctle_record[0]
        print "   CTLE search ISI1 =", ctle_isi[0]
        print "   Freq accu before CTLE search =", ctle_freq_accu
        print "   Freq accu after CTLE search =", ctle_freq_accu1
        if (ctle_record[1]!=-1):
            print "   CTLE after search 1 =", ctle_record[1]
            print "   CTLE search ISI2 =", ctle_isi[1]
        else:
            print "   CTLE search2 is skipped"
        print "   Final CTLE =", ctle
        print "=== Delta search ==="
        print "   Searched %d times. Search process:" % (delta_times0), delta_dump0
        print "   F(-1) =", fm1_dump0

    print "FFE adapt:"
    for i in range(1):
        print "FFE adapt iteration %d: [%d, %d, %d, %d], nbias=%d" % (i,
                ffe_adapt[i][0], ffe_adapt[i][1], ffe_adapt[i][2], ffe_adapt[i][3], nbias_adapt[i])


    # timers
    timers = BE_debugs(lane, 2, range(10, 18))
    t0 = timers[1]
    timers = [x-t0 for x in timers]
    timers = [timers[0]] + [x if x>=0 else x+65536 for x in timers[1:]]
    start_time, sd_time, ca_done_time, dc_search_time, link_time, ctle_search_time, delta_search_time, done_time = timers
    print "Timers: "
    print "Start time: %d" % start_time
    print "SD time: %d" % sd_time
    print "CA done: %d" % ca_done_time
    print "DC search: %d" % dc_search_time
    print "Initial link: %d" % link_time
    print "CTLE search: %d" % ctle_search_time
    print "Delta search1: %d" % delta_search_time
    print "Done: %d" % done_time    
    
def pam4_dump_fw_ver_1(lane):
    pam4_state = BE_debug(lane, 2, 0)
    error_exit = False
    if pam4_state==0xeeef:
        error_exit = True
        pam4_state = BE_debug(lane, 0, 3)
    agcgain1 = BE_debug(lane, 2, 23)
    agcgain2 = BE_debug(lane, 2, 24)
    of_cnt = BE_debugs(lane, 2, range(400, 400+8))
    hf_cnt = BE_debugs(lane, 2, range(408, 408+8))
    agcgain1_dc1 = BE_debug(lane, 2, 8)
    agcgain2_dc1 = BE_debug(lane, 2, 9)
    final_of = BE_debug(lane, 2, 4)
    final_hf = BE_debug(lane, 2, 5)
    dc_search1_cnt3 = BE_debugs(lane, 2, range(220, 220+16))
    dc_search2_cnt3 = BE_debugs(lane, 2, range(220+16, 220+16*2))
    dc_search3_cnt3 = BE_debugs(lane, 2, range(220+16*2, 220+16*3))

    dc_search1_cnt0 = BE_debugs(lane, 2, range(270, 270+16))
    dc_search2_cnt0 = BE_debugs(lane, 2, range(270+16, 270+16*2))
    dc_search3_cnt0 = BE_debugs(lane, 2, range(270+16*2, 270+16*3))

    dc_search1_agcgain = BE_debugs(lane, 2, range(420, 420+16))
    dc_search2_agcgain = BE_debugs(lane, 2, range(420+16, 420+16*2))
    dc_search3_agcgain = BE_debugs(lane, 2, range(420+16*2, 420+16*3))
    ratio = BE_debug(lane, 2, 2)
    agc_index = BE_debug(lane, 2, 1)
    restart_count = BE_debug(lane, 2, 7)
    delta = BE_debug_signed(lane, 2, 27)
    delta_times0 = BE_debug_signed(lane, 2, 35)+1
    delta_times1 = BE_debug_signed(lane, 2, 36)+1
    delta_dump0 = [BE_debug_signed(lane, 2, 110+i) for i in range(20)]
    delta_dump1 = [BE_debug_signed(lane, 2, 130+i) for i in range(20)]
    fm1_dump0 = [BE_debug_signed(lane, 2, 150+i) for i in range(20)]
    fm1_dump1 = [BE_debug_signed(lane, 2, 170+i) for i in range(20)]
    #em_dump = [BE_debug(lane, 2, 100+i) for i in range(5)]
    #ctle_sequence = BE_debug32(lane, 2, 29)
    #ctle_order = BE_debug32(lane, 2, 31)
    ctle = BE_debug(lane, 2, 33)
    next_f13 = BE_debug(lane, 2, 34)
    dc_search2_agcgain = [(x>>8, x&0xff) for x in dc_search2_agcgain]
    dc_search1_agcgain = [(x>>8, x&0xff) for x in dc_search1_agcgain]
    dc_search3_agcgain = [(x>>8, x&0xff) for x in dc_search3_agcgain]

    ctle_isi1 = [BE_debug_signed(lane, 2, 370+i) for i in range(4)]
    ctle_isi2 = [BE_debug_signed(lane, 2, 374+i) for i in range(4)]
    ffe_isi = [BE_debug_signed(lane, 2, 340+i) for i in range(4)]
    ffe_orig = [BE_debug_signed(lane, 2, 350+i) for i in range(4)]
    ffe_new = [BE_debug_signed(lane, 2, 360+i) for i in range(4)]
    smart_check_result = [BE_debug_signed(lane, 2, 390+i) for i in range(5)]
    smart_check_ths = [
            [BE_debug_signed(lane, 2, 500+j*12+i) for i in range(12)]
            for j in range(5)
            ]
    ffe_adapt = [
            [BE_debug_signed(lane, 2, 600+j*4+i) for i in range(4)]
            for j in range(10)
            ]
    nbias_adapt = [BE_debug(lane, 2, 640+i) for i in range(10)]
    if error_exit:
        print "Error exited"
    print "PAM4 state = 0x%04x" % pam4_state
    print "restart count = %d" % restart_count
    print "AGCgain = %d, %d" % (agcgain1, agcgain2)
    print "DC search1:"
    print " + cnt3 =", dc_search1_cnt3
    print " + cnt0 =", dc_search1_cnt0
    print " + agcgain =", dc_search1_agcgain
    print "AGCgain after DC search1 = %d, %d" % (agcgain1_dc1, agcgain2_dc1)
    print "OF: %d, OF_cnt =" % final_of, of_cnt
    print "HF: %d, HF_cnt =" % final_hf, hf_cnt
    print "Ratio = %d (%5.3f)" % (ratio, ratio/256.0)
    print "DC search2:"
    print " + cnt3 =", dc_search2_cnt3
    print " + cnt0 =", dc_search2_cnt0
    print " + agcgain =", dc_search2_agcgain
    print "Delta search 1:"
    print "+ Searched %d times. Search process:" % (delta_times0), delta_dump0
    print "+ F(-1) =", fm1_dump0
    print "CTLE search index = %d" % (agc_index)
    print "CTLE search ISI1 =", ctle_isi1
    print "CTLE search ISI2 =", ctle_isi2
    print "DC search3:"
    print " + cnt3 =", dc_search3_cnt3
    print " + cnt0 =", dc_search3_cnt0
    print " + agcgain =", dc_search3_agcgain
    print "Delta search 2:"
    print "+ Searched %d times. Search process:" % (delta_times1), delta_dump1
    print "+ F(-1) =", fm1_dump1
    print "Delta = %d" % delta
    #print "EM =", em_dump
    #print "CTLE_sequence = %08x" % ctle_sequence
    #print "CTLE_order = %08x" % ctle_order
    print "Next F1/3 = %d" % next_f13
    print "FFE isi =", ffe_isi
    print "FFE original =", ffe_orig
    print "FFE new =", ffe_new
    print "Smart check result =", smart_check_result
    for i in range(5):
        print "Smart check ths[%d] =" % i, smart_check_ths[i]
    print "FFE adapt:"
    for i in range(10):
        print "FFE adapt iteration %d: [%d, %d, %d, %d], nbias=%d" % (i,
                ffe_adapt[i][0], ffe_adapt[i][1], ffe_adapt[i][2], ffe_adapt[i][3], nbias_adapt[i])


    # timers
    timers = BE_debugs(lane, 2, range(10, 21))
    t0 = timers[1]
    timers = [x-t0 for x in timers]
    timers = [timers[0]] + [x if x>=0 else x+65536 for x in timers[1:]]
    start_time, sd_time, ca_done_time, dc_search_time, link_time, delta_search_time, ctle_search_time, delta_search2_time, ffe_search_time, done_time, link2_time = timers
    print "Timers: "
    print "Start time: %d" % start_time
    print "SD time: %d" % sd_time
    print "CA done: %d" % ca_done_time
    print "DC search: %d" % dc_search_time
    print "Initial link: %d" % link_time
    print "Delta search1: %d" % delta_search_time
    print "CTLE search: %d" % ctle_search_time
    print "2nd link: %d" % link2_time
    print "Delta search2: %d" % delta_search2_time
    print "FFE search: %d" % ffe_search_time
    print "Done: %d" % done_time

def pam4_dump_chip(lane):
    #dac_val, en = dac_pam4(lane)
    dac_val = dac(lane)[lane]
    #ctle_val = ctle_pam4(lane)
    ctle_val = ctle(lane)[lane]
    #agcgain1, agcgain2 = agcgain(lane)
    agcgain1, agcgain2, g3,g4 = dc_gain(lane)[lane]
    
    f1over3 = f13(lane)[lane]
    print "DAC = %d%s" % (dac_val, " (forced)" if en else "")
    print "CTLE = %d" % ctle_val
    print "AGCgain = %d, %d" % (agcgain1, agcgain2)
    print "F1/3 = %d" % f1over3    
#################################################################################################### 
def fw_adapt_dump(lane=None):
    lanes = get_lane_list(lane)
    for ln in lanes:
        get_lane_mode(ln)
        #c = Pam4Reg if lane_mode_list[ln] == 'pam4' else NrzReg
        if lane_mode_list[ln] == 'pam4':
            print("\n...Lane %d: PAM4 Adaptation Info...."%ln)
            pam4_dump_fw_linear_fit(ln)
        else:
            print("\n...Lane %d: NRZ Adaptation Info...."%ln)
            nrz_dump_fw(ln)
####################################################################################################
def fw_info_dump(lane=None):
    lanes = get_lane_list(lane)
    for ln in lanes:
        get_lane_mode(ln)
        #c = Pam4Reg if lane_mode_list[ln] == 'pam4' else NrzReg
        if lane_mode_list[ln] == 'pam4':
            print("\n...Lane %d: PAM4 ISI and FFE Info\n"%ln)
            pam4_info_fw(ln)
        else:
            print("\n*** Lane %d: ISI and FFE Info Available for PAM4 Lane Only! ***\n"%ln)
#################################################################################################### End of FW DUMP

def bitmux_status(print_en=True, fifo_check_en=1):
    bitmux_return_status=True
    if fifo_check_en: # [Acceptable range for each FIFO pointer]
        buf_ptr_min_limit = [1,  13]
        buf_ptr_max_limit = [1,  13]
        buf_ptr_del_limit = [0,  10]        
    else:       
        buf_ptr_min_limit = [0,  15]
        buf_ptr_max_limit = [0,  15]
        buf_ptr_del_limit = [0,  10]
    
    buf_type = ['--Split-->','       -->','       ---','<--Merge--']      
    print("\n -- BitMux Buffers: Min,Curr,Max,Delta")
    for a_side_buf in range(0,4):
        wreg([0x984d,[14,12]], a_side_buf)
        for b_side_buf in range(4,8):
            wreg([0x984d,[10,8]], b_side_buf)
            buf_ptr_min  = rreg([0x984e, [11,8]])
            buf_ptr_max  = rreg([0x984e,  [7,4]])
            buf_ptr_cur  = rreg([0x984e,[15,12]])
            buf_ptr_del  = buf_ptr_max - buf_ptr_min
            
            buf_ptr_min_status = 1 if  buf_ptr_min_limit[0] <= buf_ptr_min <= buf_ptr_min_limit[1] else -1
            buf_ptr_max_status = 1 if  buf_ptr_max_limit[0] <= buf_ptr_max <= buf_ptr_max_limit[1] else -1
            buf_ptr_del_status = 1 if  buf_ptr_del_limit[0] <= buf_ptr_del <= buf_ptr_del_limit[1] else -1
            buf_ptr_cur_status = 1 if  buf_ptr_del_limit[0] <= buf_ptr_cur <= buf_ptr_del_limit[1] else -1
            status_flag = '<<' if buf_ptr_min_status<0 or buf_ptr_max_status<0 or buf_ptr_del_status<0 else ' '
            if status_flag=='<<':
                bitmux_return_status=False
            print("\n A%d %s B%d :"%(a_side_buf,buf_type[b_side_buf-4],a_side_buf*2+(b_side_buf%2))),
            print("%3d %3d  %3d %3d"%(buf_ptr_min,buf_ptr_cur,buf_ptr_max, buf_ptr_del)),
            print("%s"%(status_flag)),
        print("")

    return bitmux_return_status

def fw_bitmux_status(print_en=True, fifo_check_en=1):
    return_status=True
    return_retries_config=0
    return_retries_runtime=0
    if fifo_check_en: # [Acceptable range for each FIFO pointer]
        buf_ptr_min_limit = [2,  13]
        buf_ptr_max_limit = [2,  13]
        buf_ptr_del_limit = [1,  10]        
    else:       
        buf_ptr_min_limit = [0,  15]
        buf_ptr_max_limit = [0,  15]
        buf_ptr_del_limit = [0,  10]
    
    buf_type = ['--Split-->','       -->','       ---','<--Merge--']
    for a_side_buf in range(0,4):
    
        bitmux_state  = BE_debug(a_side_buf, 4, 0)           # Bitmux state (5 means 'Done with bitmux config')
        bitmux_retries_config  = BE_debug(a_side_buf, 4, 24) # Bitmux FIFO retry counter during bitmux config
        bitmux_retries_runtime = BE_debug(a_side_buf, 4, 25) # Bitmux FIFO retry counter after bitmux config (runtime FIFO check)
       
        print("\n FW BitMux Buffers: Min,Max,Delta [Min,Max,Delta]"),
        bitmux_state_char = '=' if bitmux_state==5 else '->'
        print(",State%s %d" % (bitmux_state_char,bitmux_state)),
        bitmux_retry_char = '=' if (bitmux_retries_config==0 and bitmux_retries_runtime==0) else '->'
        print(",Retries%s %d %d" % (bitmux_retry_char, bitmux_retries_config,bitmux_retries_runtime)),
        if bitmux_retries_config > return_retries_config:
            return_retries_config = bitmux_retries_config
        if bitmux_retries_runtime > return_retries_runtime:
            return_retries_runtime = bitmux_retries_runtime
                
        for b_side_buf in range(4,8):
            buf_ptr_max  = BE_debug(a_side_buf, 4, (b_side_buf)+6)
            buf_ptr_min  = BE_debug(a_side_buf, 4, (b_side_buf)+10)
            buf_ptr_cur  = 0
            buf_ptr_del  = buf_ptr_max - buf_ptr_min
            
            buf_ptr_max_during_config  = BE_debug(a_side_buf, 4, (b_side_buf)+26)
            buf_ptr_min_during_config  = BE_debug(a_side_buf, 4, (b_side_buf)+30)
            buf_ptr_del_during_config  = buf_ptr_max_during_config - buf_ptr_min_during_config
            
            buf_ptr_min_status = 1 if  buf_ptr_min_limit[0] <= buf_ptr_min <= buf_ptr_min_limit[1] else -1
            buf_ptr_max_status = 1 if  buf_ptr_max_limit[0] <= buf_ptr_max <= buf_ptr_max_limit[1] else -1
            buf_ptr_del_status = 1 if  buf_ptr_del_limit[0] <= buf_ptr_del <= buf_ptr_del_limit[1] else -1
            buf_ptr_cur_status = 1 if  buf_ptr_del_limit[0] <= buf_ptr_cur <= buf_ptr_del_limit[1] else -1
            status_flag = '<<' if buf_ptr_min_status<0 or buf_ptr_max_status<0 or buf_ptr_del_status<0 else ' '
            
            print("\n A%d %s B%d :"%(a_side_buf,buf_type[b_side_buf-4],a_side_buf*2+(b_side_buf%2))),
            print("%3d %3d %3d  "%(buf_ptr_min,buf_ptr_max,buf_ptr_del)),
            print("[%3d %3d %3d  ] "%(buf_ptr_min_during_config,buf_ptr_max_during_config, buf_ptr_del_during_config)),
            print("%s"%(status_flag)),
            if status_flag=='<<':
                return_status=False


        print("")

    return return_status, return_retries_config, return_retries_runtime


def pll_cal_dump():
    cal_done_mask = BE_debug(0, 0, 5)
    print "Calibration done mask: %04x" % cal_done_mask
    if cal_done_mask==0: return
    print "Lane CalStart CalDone Time Count TxCap RxCap Txmin Txmax RXmin RXmax"
    for lane in range(16):
        if (cal_done_mask & (1<<lane))==0:
            continue
        cal_start = BE_debug(lane, 0, 6)
        cal_done = BE_debug(lane, 0, 7)
        cal_count = BE_debug(lane, 0, 8)
        cal_time = cal_done - cal_start
        txcap = (MdioRd(0x7000+lane*512+0xdb)>>8 ) & 0x7f
        rxcap = (MdioRd(0x7000+lane*512+0x1f5)>>9 ) & 0x7f
        txmin = BE_debug(lane, 0, 10)
        txmax = BE_debug(lane, 0, 11)
        rxmin = BE_debug(lane, 0, 12)
        rxmax = BE_debug(lane, 0, 13)
        if (cal_time<0): cal_time += 1<<16
        print " %2d  %5d   %5d  %5d  %d    %3d   %3d   %3d   %3d   %3d   %3d  " % (
                lane, cal_start, cal_done, cal_time, cal_count, txcap, rxcap,
                txmin, txmax, rxmin, rxmax)
    
def pll_cal_debug() : 
    retry_cnt = {}
    rx_start_time = {}
    rx_end_time = {}
    tx_end_time = {}
    rx_val = {}
    tx_val = {}
    
    for x in range(0,16) :
        wreg(0x9807, 0x0005)
        wreg(0x9806, 0xb000+x)
        time.sleep(.1)
        retry_cnt[x] = rreg(0x9807)
        time.sleep(.1)
        wreg(0x9807, 0x0006)
        wreg(0x9806, 0xb000+x)
        time.sleep(.1)
        rx_start_time[x] = rreg(0x9807)
        time.sleep(.1)
        wreg(0x9807, 0x0007)
        wreg(0x9806, 0xb000+x)
        time.sleep(.1)
        rx_end_time[x] = rreg(0x9807)
        time.sleep(.1)
        wreg(0x9807, 0x0008)
        wreg(0x9806, 0xb000+x)
        time.sleep(.1)
        tx_end_time[x] = rreg(0x9807)
        time.sleep(.1)
        wreg(0x9807, 0x0009)
        wreg(0x9806, 0xb000+x)
        time.sleep(.1)
        rx_val[x] = rreg(0x9807)
        time.sleep(.1)
        wreg(0x9807, 0x000a)
        wreg(0x9806, 0xb000+x)
        time.sleep(.1)
        tx_val[x] = rreg(0x9807)
        time.sleep(.1)

    for x in range(0,16) :
        print("lane[%d] : retry->%d, rx_start->%d, rx_end->%d, tx_end->%d, rx_val->%d, tx_val->%d" % (x, retry_cnt[x], rx_start_time[x], rx_end_time[x], tx_end_time[x], rx_val[x], tx_val[x]))
####################################################################################################
# 
# Initialize and adapt lane(s) in NRZ or PAM4 mode
#
# Note: this function does a complete initialization and adaptation of lane in a NRZ/PAM4 mode
#       It optimizes the RX of the lane to the connected channel
####################################################################################################
def test_slice_prbs(linecard_num=0,phy_num=0,slice_num=None, pam4_lanes = range(4), nrz_lanes = range(8,16), rst=0, ber_time = 10, header=1, logfile='rx_monitor_log.txt'):

    if slice_num==None: 
        slice_num = gSlice
      
    lanes = pam4_lanes + nrz_lanes
    
    if header:
        log_file = open(logfile, 'a+')
        log_file.write('\n#######################################################################################################'),
        log_file.write("\n#....Testing PRBS BER, %s" % time.asctime()),
        log_file.write("\n#....Linecard,Phy,Slice: [%2d,%2d,%2d]"%(linecard_num,phy_num,slice_num)),
        log_file.write("\n#....Lanes:"),
        log_file.write(str(pam4_lanes)),
        log_file.write(str(nrz_lanes)),
        log_file.write('\n#######################################################################################################'),
        log_file.close()
    
    print("\n....Linecard/Phy/Slice [%d, %d, %d]; Lanes"%(linecard_num,phy_num,slice_num)),
    print(pam4_lanes),
    print(nrz_lanes),
    if rst==1:
        print("; Adapting..."),
        init_lane(mode='pam4',lane=pam4_lanes)
        init_lane(mode='nrz', lane=nrz_lanes)    
        fw_config_lane(mode='pam4',lane=pam4_lanes)
        fw_config_lane(mode='nrz', lane=nrz_lanes)
        [adapt_done, max_adapt_time] = fw_adapt_wait (lane=lanes, print_en=0)
        print("\b\b\b\b\b\b\bed in %4.1fs"%(max_adapt_time)),
    else:
        print("; Adaptation skipped"),
        max_adapt_time=-1
    
    if header:
        #serdes_params(lane=lanes)
        log_file = open(logfile, 'a+')
        temp_stdout = sys.stdout
        sys.stdout = log_file
        serdes_params(lanes)
        sys.stdout = temp_stdout
        log_file.close()
    
    if rst==1:
        print("; BER for %4.1fs..."%(ber_time)),
        rx_monitor(rst=1, lane=lanes,print_en=0)
        time.sleep(ber_time)
        #print("Done"),

    post_fec_ber , ber_time = rx_monitor_log(linecard_num=linecard_num,phy_num=phy_num,slice_num=slice_num,lane=lanes, rst=0,adapt_time=max_adapt_time, header=header,logfile=logfile)
    print("\b\b\b\b; Results Saved to %s"%(logfile)),
    print("\n....Post-FEC BER for %1.1fs, PAM4:["%(ber_time)),
    for ln in pam4_lanes: 
        if post_fec_ber[ln] == 0:
            print ("%d," % (post_fec_ber[ln]) ),
        else:
            print ("%1.1e," % (post_fec_ber[ln]) ),
    print("]  NRZ:["),
    for ln in nrz_lanes: 
        if post_fec_ber[ln] == 0:
            print ("%d," % (post_fec_ber[ln]) ),
        else:
            print ("%1.1e," % (post_fec_ber[ln]) ),
    print("]\n")

####################################################################################################
# 
# PAM4 Phase Error Functions
####################################################################################################    
def phase_error_sel_ph(phase=0,lane=0):
    if phase==0:
        wregBits(0x6e,[7,0],0x0,lane)
    else:
        wregBits(0x6e,[7,0],0xff-(0x1<<(phase-1)),lane)
    
def phase_error_err_cntr(t=5,lane=0,print_en=0):
    cntr_1_i = rregBits(0x53,[15,0],lane) + (rregBits(0x52,[15,0],lane)<<16)
    cntr_2_i = rregBits(0x55,[15,0],lane) + (rregBits(0x54,[15,0],lane)<<16)
    cntr_3_i = rregBits(0x57,[15,0],lane) + (rregBits(0x56,[15,0],lane)<<16)
    cntr_4_i = rregBits(0x59,[15,0],lane) + (rregBits(0x58,[15,0],lane)<<16)
    cntr_5_i = rregBits(0x5b,[15,0],lane) + (rregBits(0x5a,[15,0],lane)<<16)
    cntr_6_i = rregBits(0x5d,[15,0],lane) + (rregBits(0x5c,[15,0],lane)<<16)
    cntr_7_i = rregBits(0x5f,[15,0],lane) + (rregBits(0x5e,[15,0],lane)<<16)
    cntr_8_i = rregBits(0x61,[15,0],lane) + (rregBits(0x60,[15,0],lane)<<16)
    cntr_9_i = rregBits(0x63,[15,0],lane) + (rregBits(0x62,[15,0],lane)<<16)
    cntr_a_i = rregBits(0x65,[15,0],lane) + (rregBits(0x64,[15,0],lane)<<16)
    cntr_b_i = rregBits(0x67,[15,0],lane) + (rregBits(0x66,[15,0],lane)<<16)
    cntr_c_i = rregBits(0x69,[15,0],lane) + (rregBits(0x68,[15,0],lane)<<16)

    time.sleep(t)

    cntr_1_p = rregBits(0x53,[15,0],lane) + (rregBits(0x52,[15,0],lane)<<16)
    cntr_2_p = rregBits(0x55,[15,0],lane) + (rregBits(0x54,[15,0],lane)<<16)
    cntr_3_p = rregBits(0x57,[15,0],lane) + (rregBits(0x56,[15,0],lane)<<16)
    cntr_4_p = rregBits(0x59,[15,0],lane) + (rregBits(0x58,[15,0],lane)<<16)
    cntr_5_p = rregBits(0x5b,[15,0],lane) + (rregBits(0x5a,[15,0],lane)<<16)
    cntr_6_p = rregBits(0x5d,[15,0],lane) + (rregBits(0x5c,[15,0],lane)<<16)
    cntr_7_p = rregBits(0x5f,[15,0],lane) + (rregBits(0x5e,[15,0],lane)<<16)
    cntr_8_p = rregBits(0x61,[15,0],lane) + (rregBits(0x60,[15,0],lane)<<16)
    cntr_9_p = rregBits(0x63,[15,0],lane) + (rregBits(0x62,[15,0],lane)<<16)
    cntr_a_p = rregBits(0x65,[15,0],lane) + (rregBits(0x64,[15,0],lane)<<16)
    cntr_b_p = rregBits(0x67,[15,0],lane) + (rregBits(0x66,[15,0],lane)<<16)
    cntr_c_p = rregBits(0x69,[15,0],lane) + (rregBits(0x68,[15,0],lane)<<16)

    #cntr = cntr_p - cntr_i
    cntr_1 = cntr_1_p - cntr_1_i
    cntr_2 = cntr_2_p - cntr_2_i
    cntr_3 = cntr_3_p - cntr_3_i
    cntr_4 = cntr_4_p - cntr_4_i
    cntr_5 = cntr_5_p - cntr_5_i
    cntr_6 = cntr_6_p - cntr_6_i
    cntr_7 = cntr_7_p - cntr_7_i
    cntr_8 = cntr_8_p - cntr_8_i
    cntr_9 = cntr_9_p - cntr_9_i
    cntr_a = cntr_a_p - cntr_a_i
    cntr_b = cntr_b_p - cntr_b_i
    cntr_c = cntr_c_p - cntr_c_i
    cntr = cntr_1+cntr_2+cntr_3+cntr_4+cntr_5+cntr_6+cntr_7+cntr_8+cntr_9+cntr_a+cntr_a+cntr_b+cntr_c
    #div = (cntr_1+cntr_2+cntr_3+cntr_4+cntr_5+cntr_6+cntr_7+cntr_8+cntr_9+cntr_a+cntr_a+cntr_b+cntr_c)/float(cntr)*100.0

    if (print_en==1):
        print 'prbs check error: ',cntr
        print 'cntr 1: ',cntr_1
        print 'cntr 2: ',cntr_2
        print 'cntr 3: ',cntr_3
        print 'cntr 4: ',cntr_4
        print 'cntr 5: ',cntr_5
        print 'cntr 6: ',cntr_6
        print 'cntr 7: ',cntr_7
        print 'cntr 8: ',cntr_8
        print 'cntr 9: ',cntr_9
        print 'cntr a: ',cntr_a
        print 'cntr b: ',cntr_b
        print 'cntr c: ',cntr_c
        #print 'cntr/prbs_check_err : %4.1f%%'%(div)

    return [cntr,cntr_1,cntr_2,cntr_3,cntr_4,cntr_5,cntr_6,cntr_7,cntr_8,cntr_9,cntr_a,cntr_b,cntr_c]

def phase_error_prev_func(en=0,prev=0,lane=0):
    wregBits(0x45,[9],en,lane)
    wregBits(0x45,[8,7],prev,lane) 

def phase_error(t=5,print_en=1,lane=0):
        data = {}
        phase_error_prev_func(en=1,prev=0,lane=lane)
        for phase in range(9):
            phase_error_sel_ph(phase,lane)
            data[phase]=phase_error_err_cntr(t,lane,print_en=0)
        if (print_en==1):
            print '\n---------------------------------------------------------- Prev 00 ----------------------------------------------------------------',
            print '\nPrbsErrCntr:     total,    0001,    0010,    0011,    0100,    0110,    0111,    1000,    1001,    1011,    1100,    1101,    1110,',
            for phase in range(9):
                if phase==0:
                    print '\nphase all:',
                else:
                    print '\nphase %3X:'%(phase),
                for err in range(len(data[phase])):
                    if err==0:print '%11d,'%(data[phase][err]),
                    else:print '%7d,'%(data[phase][err]),

        data = {}
        phase_error_prev_func(en=1,prev=1,lane=lane)
        for phase in range(9):
            phase_error_sel_ph(phase,lane)
            data[phase]=phase_error_err_cntr(t,lane,print_en=0)
        if (print_en==1):
            print '\n---------------------------------------------------------- Prev 01 ----------------------------------------------------------------',
            print '\nPrbsErrCntr:     total,    0001,    0010,    0011,    0100,    0110,    0111,    1000,    1001,    1011,    1100,    1101,    1110,',
            for phase in range(9):
                if phase==0:
                    print '\nphase all:',
                else:
                    print '\nphase %3X:'%(phase),
                for err in range(len(data[phase])):
                    if err==0:print '%11d,'%(data[phase][err]),
                    else:print '%7d,'%(data[phase][err]),

        data = {}
        phase_error_prev_func(en=1,prev=2,lane=lane)
        for phase in range(9):
            phase_error_sel_ph(phase,lane)
            data[phase]=phase_error_err_cntr(t,lane,print_en=0)
        if (print_en==1):
            print '\n---------------------------------------------------------- Prev 10 ----------------------------------------------------------------',
            print '\nPrbsErrCntr:     total,    0001,    0010,    0011,    0100,    0110,    0111,    1000,    1001,    1011,    1100,    1101,    1110,',
            for phase in range(9):
                if phase==0:
                    print '\nphase all:',
                else:
                    print '\nphase %3X:'%(phase),
                for err in range(len(data[phase])):
                    if err==0:print '%11d,'%(data[phase][err]),
                    else:print '%7d,'%(data[phase][err]),

        data = {}
        phase_error_prev_func(en=1,prev=3,lane=lane)
        for phase in range(9):
            phase_error_sel_ph(phase,lane)
            data[phase]=phase_error_err_cntr(t,lane,print_en=0)
        if (print_en==1):
            print '\n---------------------------------------------------------- Prev 11 ----------------------------------------------------------------',
            print '\nPrbsErrCntr:     total,    0001,    0010,    0011,    0100,    0110,    0111,    1000,    1001,    1011,    1100,    1101,    1110,',
            for phase in range(9):
                if phase==0:
                    print '\nphase all:',
                else:
                    print '\nphase %3X:'%(phase),
                for err in range(len(data[phase])):
                    if err==0:print '%11d,'%(data[phase][err]),
                    else:print '%7d,'%(data[phase][err]),
####################################################################################################
def read_comp_one(dir=0, phase=0,addr=1, lane= 0):

    off_ctl_addr     = 0x0E8
    off_ctl_dir_loc  = [12]
    off_ctl_addr_loc = [11,8]
    off_ctl_ph_loc   = [7,6]

    off_val_addr     = 0x0DF
    off_val_calp_loc = [15,10]
    off_val_caln_loc = [9,4]


    wreg([off_ctl_addr, off_ctl_dir_loc],   dir, lane)
    wreg([off_ctl_addr, off_ctl_addr_loc], addr, lane)
    wreg([off_ctl_addr, off_ctl_ph_loc],   phase, lane)

    calp = rreg([off_val_addr,off_val_calp_loc],lane)
    caln = rreg([off_val_addr,off_val_caln_loc],lane)
    
    wreg([off_ctl_addr,off_ctl_dir_loc],  0, lane)
    wreg([off_ctl_addr,off_ctl_addr_loc], 0, lane)
    wreg([off_ctl_addr,off_ctl_ph_loc],   0, lane)
    
    return [calp,caln,abs(calp-caln)]
        
####################################################################################################
def read_comp_all(lane=None, print_en=1):

    off_ctl_addr     = 0x0E8
    off_ctl_dir_loc  = [12]
    off_ctl_addr_loc = [11,8]
    off_ctl_ph_loc   = [7,6]

    off_val_addr     = 0x0DF
    off_val_calp_loc = [15,10]
    off_val_caln_loc = [9,4]
    
    lanes = get_lane_list(lane)
    
    cal_p_min_max= [[100,-1] for _ in range(16)]
    cal_n_min_max= [[100,-1] for _ in range(16)]
    cal_d_min_max= [[100,-1] for _ in range(16)]
    
    ##### Print Headers
    print("\n  Slice_%d Comparator Offset %s:"%(gSlice, '{calp,caln) Diff' if print_en==2 else 'Pos to Neg Differences')),
    line_separator= "\n +---------------+%s%s"%('------------'*len(lanes) if print_en==2 else '----'*len(lanes), '-'*(int((len(lanes)/4)+.5)) if len(lanes)>4 else '')# if len(lanes)!=16 else "\n +-----------------------------------------------------------------------------------+"
    print line_separator,
    print("\n |Num,Dir,Ph,Cmp |"),
    for ln in lanes:
        if print_en==2: print("(C+,C-)"),
        print("%2s%s"%(lane_name_list[ln],',' if (ln+1)%4 else " |")),
    print line_separator,

    ##### Print values
    comp_num=0
    for dir in range(0,2):
        for phase in range(0,4):
            for addr in range(1,16):
                comp_num+=1; print("\n |%3d,%3d,%2d,%3d |"%(comp_num, dir,phase,addr)),
                for ln in lanes:
                    cal_p_samples = []
                    cal_n_samples = []
                    cal_d_samples = []
                    ##### Take the average of 10 readings
                    for i in range(10):
                        #[cal_p,cal_n,cal_d] = read_comp_one(dir,phase,addr,ln)
                        wreg([off_ctl_addr, off_ctl_dir_loc],   dir, ln)
                        wreg([off_ctl_addr, off_ctl_addr_loc], addr, ln)
                        wreg([off_ctl_addr, off_ctl_ph_loc],  phase, ln)

                        cal_p = rreg([off_val_addr,off_val_calp_loc],ln)
                        cal_n = rreg([off_val_addr,off_val_caln_loc],ln)
                        cal_d=abs(cal_p-cal_n)
                        
                        if cal_p < cal_p_min_max[ln][0]: cal_p_min_max[ln][0]=cal_p
                        if cal_n < cal_n_min_max[ln][0]: cal_n_min_max[ln][0]=cal_n
                        if cal_d < cal_d_min_max[ln][0]: cal_d_min_max[ln][0]=cal_d
                        if cal_p > cal_p_min_max[ln][1]: cal_p_min_max[ln][1]=cal_p
                        if cal_n > cal_n_min_max[ln][1]: cal_n_min_max[ln][1]=cal_n
                        if cal_d > cal_d_min_max[ln][1]: cal_d_min_max[ln][1]=cal_d
                        cal_p_samples.append(cal_p)
                        cal_n_samples.append(cal_n)
                        cal_d_samples.append(cal_d)
                        
                    wreg([off_ctl_addr, off_ctl_dir_loc],   0, ln)
                    wreg([off_ctl_addr, off_ctl_addr_loc],  0, ln)
                    wreg([off_ctl_addr, off_ctl_ph_loc],    0, ln)
                    
                    cal_p_avg = sum(cal_p_samples)/len(cal_p_samples)
                    cal_n_avg = sum(cal_n_samples)/len(cal_n_samples)
                    cal_d_avg = sum(cal_d_samples)/len(cal_d_samples)

                    if print_en==2: print("(%2d,%2d)"%(cal_p_avg,cal_n_avg)),
                    print("%2d%s"%(cal_d_avg,',' if (ln+1)%4 else " |")),
 
    ##### Print min/max offset values for each lane
    print line_separator,
    print("\n |Min Comp Offset|"),
    for ln in lanes:
        if print_en==2: print("(%2d,%2d)"%(cal_p_min_max[ln][0],cal_n_min_max[ln][0])),
        print("%2d%s"%(cal_d_min_max[ln][0],',' if (ln+1)%4 else " |")),
    print("\n |Max Comp Offset|"),
    for ln in lanes:
        if print_en==2: print("(%2d,%2d)"%(cal_p_min_max[ln][1],cal_n_min_max[ln][1])),
        print("%2d%s"%(cal_d_min_max[ln][1],',' if (ln+1)%4 else " |")),
    print line_separator,

####################################################################################################
# 
# GET ISI Residual Taps for NRZ
####################################################################################################
def sw_nrz_isi(lane=None,print_en=0):
    isi_tap_range=range(0,16)
    if lane==None: lane=gLane[0]
    result={}
    if fw_loaded:
        #print("...sw_nrz_isi: Slice %d Lane %2d has FW Loaded. Exiting!"%(gSlice,lane))
        result[lane] = [-1]*len(isi_tap_range)
        return result
    if type(lane) != int:
        print "...sw_nrz_isi: This is a single-lane function"
        result[lane] = [-1]*len(isi_tap_range)
        return result

    bp1_ori = rregBits(0x10b, [12, 8],lane)
    bp1_en_ori = rregBits(0x10b, [15],lane)
    wregBits(0x10b, [12, 8], 24,lane)
    tap_list = []
    wregBits(0x10b, [15], 0, lane)
    wregBits(0x10c, [15], 0, lane)
    wregBits(0x10c, [15], 1, lane)

    for i in isi_tap_range:
        wregBits(0x165, [4,1],i,lane)
        time.sleep(0.01)
        #wregBits_nrz(0x0b, [15], 0)
        wregBits(0x10b, [15], 1, lane)

        wait_for_bp1_timeout=0
        while True:
            if rregBits(0x10d, [15],lane):
                break
            else:
                wait_for_bp1_timeout+=1
                if wait_for_bp1_timeout>5000:
                    if print_en==2:print " Get Tap Value >>>>> Timed out 2 waiting for BP1"
                    break
        plus = (rregBits(0x127, [3,0],lane)<<8) + rregBits(0x128, [15,8],lane)
        minus = (rregBits(0x128, [7,0],lane)<<4) + rregBits(0x129, [15,12],lane)

        if (plus>2047): plus = plus - 4096
        if (minus>2047): minus = minus - 4096
        diff_margin = plus - minus 
        diff_margin_f = ((float(diff_margin & 0x0fff)/2048)+1)%2-1

        if print_en==2: print "\n%8d, %8d, %8d, %11d, %11.4f "  % (i, plus, minus, diff_margin, diff_margin_f),
        tap_list.append(diff_margin)

        wregBits(0x10b, [15], 0, lane)
        wregBits(0x10c, [15], 0, lane)
        wregBits(0x10c, [15], 1, lane)

    wregBits(0x10b, [12, 8], bp1_ori, lane)
    wregBits(0x10b, [15], bp1_en_ori, lane)
    result[lane] = tap_list 
    return result
    
def read_plus_minus_margin_nrz(lane=None):
    lanes = get_lane_list(lane)
    result = {}
    for lane in lanes:
        plus_0 = rregBits(0x11a,[15,4],lane)
        plus_1 = (rregBits(0x11a,[3,0],lane)<<8)+rregBits(0x11b,[15,8],lane)
        plus_2 = (rregBits(0x11b,[7,0],lane)<<4)+rregBits(0x11c,[15,12],lane)
        plus_3 = rregBits(0x11c,[11,0],lane)
        plus_4 = rregBits(0x11d,[15,4],lane)
        plus_5 = (rregBits(0x11d,[3,0],lane)<<8)+rregBits(0x11e,[15,8],lane)
        plus_6 = (rregBits(0x11e,[7,0],lane)<<4)+rregBits(0x11f,[15,12],lane)
        plus_7 = rregBits(0x11f,[11,0],lane)
        if plus_0 > 2047: plus_0 = plus_0 - 4096
        if plus_1 > 2047: plus_1 = plus_1 - 4096
        if plus_2 > 2047: plus_2 = plus_2 - 4096
        if plus_3 > 2047: plus_3 = plus_3 - 4096
        if plus_4 > 2047: plus_4 = plus_4 - 4096
        if plus_5 > 2047: plus_5 = plus_5 - 4096
        if plus_6 > 2047: plus_6 = plus_6 - 4096
        if plus_7 > 2047: plus_7 = plus_7 - 4096
        plus_margin = [plus_0,plus_1,plus_2,plus_3,plus_4,plus_5,plus_6,plus_7]
        minus_0 = rregBits(0x120,[15,4],lane)
        minus_1 = (rregBits(0x120,[3,0],lane)<<8)+rregBits(0x121,[15,8],lane)
        minus_2 = (rregBits(0x121,[7,0],lane)<<4)+rregBits(0x122,[15,12],lane)
        minus_3 = rregBits(0x122,[11,0],lane)
        minus_4 = rregBits(0x123,[15,4],lane)
        minus_5 = (rregBits(0x123,[3,0],lane)<<8)+rregBits(0x124,[15,8],lane)
        minus_6 = (rregBits(0x124,[7,0],lane)<<4)+rregBits(0x125,[15,12],lane)
        minus_7 = rregBits(0x125,[11,0],lane)
        if minus_0 > 2047: minus_0 = minus_0 - 4096
        if minus_1 > 2047: minus_1 = minus_1 - 4096
        if minus_2 > 2047: minus_2 = minus_2 - 4096
        if minus_3 > 2047: minus_3 = minus_3 - 4096
        if minus_4 > 2047: minus_4 = minus_4 - 4096
        if minus_5 > 2047: minus_5 = minus_5 - 4096
        if minus_6 > 2047: minus_6 = minus_6 - 4096
        if minus_7 > 2047: minus_7 = minus_7 - 4096
        minus_margin = [minus_0,minus_1,minus_2,minus_3,minus_4,minus_5,minus_6,minus_7]
        #print 'plus_margin=', plus_margin
        #print 'minus_margin=',minus_margin
        result[lane] = plus_margin, minus_margin
    else:
        if result != {}: return result


def v_sensor(on_chip_sensor=1, print_en=1):
    # function used to measure voltage
    # if on chip sensor deactivated, use external multimeter (currently commented out)
    Vsensor_base_addr = 0xB000

    if on_chip_sensor==0: # Use Bench Multimeter connected to laptop through GPIB/USB
        v=mm.v
        return v*1000
        pass
    else: # Use On-Chip Voltage Sensor
        
        chip.MdioWr((Vsensor_base_addr + 0x3f),0x010d) 
        chip.MdioWr((Vsensor_base_addr + 0xf6),0x1040) #sel[6:3]=8,pd[12]=1
        time.sleep(0.1)
        chip.MdioWr((Vsensor_base_addr + 0xf6),0x0040) #sel[6:3]=8,pd[12]=0
        time.sleep(0.1)
        chip.MdioWr((Vsensor_base_addr + 0xf6),0x0040) 
        time.sleep(0.1)
        chip.MdioWr((Vsensor_base_addr + 0xf6),0x0440) #sel[6:3]=8,pd[12]=0;rstn[10]=1
        time.sleep(0.1)
        chip.MdioWr((Vsensor_base_addr + 0xf6),0x0c40) #sel[6:3]=8,pd[12]=0;rstn[10]=1,run[11]=1
        time.sleep(0.1)

        k = 1
        for i in range(0,k): 
            chip.MdioWr((Vsensor_base_addr + 0xf6),0x0c40+(i<<3)) #sel[6:3]=8,pd[12]=0;rstn[10]=1,run[11]=1
            time.sleep(0.3)
            rdatah = chip.MdioRd(Vsensor_base_addr + 0xf5) 
            rdatah = (rdatah& 0x0fff) 
            #if print_en: print "Voltage sensor register data:0x%04x" % (rdatah) 
             
            rdata = ((rdatah+1.0)/ 256.0) * 1.224
        if print_en: 
            print "On-Chip Vsense Voltage: %4.0f mV" % (rdata*1000.0) 
        else: 
            return rdata*1000
####################################################################################################
#
#  SLT Commands
#
#################################################################################################### 
def slt_mode(slt_ver=None):
    global gSltVer
    if slt_ver!=None:
        gSltVer = slt_ver
        if slt_ver==1.0: # test slt_ver 1.0
            wreg(0x9505,0x9000) # changed from 0x9000 
            wreg(0x9605,0x9000) # changed from 0x9000 
            for ln in range(16):
                wreg(0x0fc,0x7236) # changed from 0x7236
                wreg(0x1fb,0x49c6) # changed from 0x49c6
                wreg([0x020,  [8,6]], 5, ln)   # Register 0x020 change from 0x03C0 to 0x0340 (PAM4 CDR BW changed from 7 to 5)
                wreg([0x079,   [13]], 1, ln)   # Register 0x079 change from 0x00A4 to 0x20A4 (PAM4 Timing qual changed from 0 to 1)
                wreg([0x13b,[14,12]], 3, ln)   # Register 0x13B change from 0xE000 to 0xB000 (NRZ CDR BW changed from 6 to 3)
                
        elif slt_ver==2.0: # test slt_ver 2.0
            wreg(0x9505,0x9c00) # changed from 0x9000 
            wreg(0x9605,0x9c00) # changed from 0x9000 
            for ln in range(16):
                wreg(0x0fc,0x73B6) # changed from 0x7236
                wreg(0x1fb,0x49f6) # changed from 0x49c6
                wreg([0x020,  [8,6]], 5, ln)   # Register 0x020 change from 0x03C0 to 0x0340 (PAM4 CDR BW changed from 7 to 5)
                wreg([0x079,   [13]], 1, ln)   # Register 0x079 change from 0x00A4 to 0x20A4 (PAM4 Timing qual changed from 0 to 1)
                wreg([0x13b,[14,12]], 3, ln)   # Register 0x13B change from 0xE000 to 0xB000 (NRZ CDR BW changed from 6 to 3)
                
        else: # default operating mode
            wreg(0x9505,0x9000) # changed from 0x9000 
            wreg(0x9605,0x9000) # changed from 0x9000 
            for ln in range(16):
                wreg(0x0fc,0x7236) # changed from 0x7236
                wreg(0x1fb,0x49c6) # changed from 0x49c6
                wreg([0x020,  [8,6]], 7, ln)   # Register 0x020 change to 0x03c0 (PAM4 CDR BW set to 7)
                wreg([0x079,   [13]], 0, ln)   # Register 0x079 change to 0x00A4 (PAM4 Timing qual set to 0)
                wreg([0x13b,[14,12]], 6, ln)   # Register 0x13B change to 0xE000 (NRZ CDR BW set to 6)

    if gSltVer > 0:
        print("\n...Slice %d SLT Version %1.1f is ENABLED..."%(gSlice, float(gSltVer)))
    else:
        print("\n...Slice %d SLT Mode is DISABLED..."%gSlice)

def rxm(rst=0):
    sel_slice(0);temp_sensor_fast();rx_monitor(rst=rst);sel_slice(1);rx_monitor(rst=rst)

def kpp(val=None,val2=None):
    sel_slice(0);
    if val2!=None: ted(val2,print_en=0)
    kp(val);
    sel_slice(1);
    if val2!=None: ted(val2,print_en=0)
    kp(val)
    

def regg(addr,val=None,lane=None):
    reg(addr,val,lane,slice=[0,1])
    
def auto_poll():
    sel_slice(0);auto_pol();sel_slice(1);auto_pol()
 
def ser():
    sel_slice(0);fw_serdes_params();sel_slice(1);fw_serdes_params()

def plll(tgt_pll=None, datarate=None, fvco=None, cap=None, n=None, div4=None, div2=None, refclk=None, frac_en=None, frac_n=None, lane=None):
    sel_slice(0);pll(tgt_pll, datarate, fvco, cap, n, div4, div2, refclk, frac_en, frac_n, lane)
    sel_slice(1);pll(tgt_pll, datarate, fvco, cap, n, div4, div2, refclk, frac_en, frac_n, lane)

def lrr():
    sel_slice(0);lr();fw_adapt_wait(max_wait=12);sel_slice(1);lr();fw_adapt_wait(max_wait=12)
    #sel_slice(0);fw_adapt_wait();sel_slice(1);fw_adapt_wait()
        

def nrz26(cap0=27, cap1=27, div2=1):
    sel_slice(0);pll(tgt_pll='both',cap=cap0,n=34,div2=0);reg(0xf3,0x8000)
    sel_slice(1);pll(tgt_pll='both',cap=cap1,n=34,div2=0);reg(0xf3,0x8000)
    if div2==1:
        sel_slice(0);pll(tgt_pll='rx',n=68,div2=1);
        sel_slice(1);pll(tgt_pll='rx',n=68,div2=1)

def nrz25(cap0=31, cap1=31, div2=1):
    sel_slice(0);pll(tgt_pll='both',cap=cap0,n=33,div2=0);reg(0xf3,0x8000)
    sel_slice(1);pll(tgt_pll='both',cap=cap1,n=33,div2=0);reg(0xf3,0x8000)
    if div2==1:
        sel_slice(0);pll(tgt_pll='rx',n=66,div2=1);
        sel_slice(1);pll(tgt_pll='rx',n=66,div2=1)

def fracn(val=80):
    sel_slice(0);reg(0xd8,val);reg(0xd7,0xa000);fw_serdes_params()
    sel_slice(1);reg(0xd8,val);reg(0xd7,0xa000);fw_serdes_params()

def fracn_tx(val=80):
    sel_slice(0);reg(0x1f1,val);reg(0x1f0,0xa000);fw_serdes_params()
    sel_slice(1);reg(0x1f1,val);reg(0x1f0,0xa000);fw_serdes_params()

def add_ppm(tx_ppm=4, rx_ppm=0, slice=[0,1], lane=None):
    lanes = get_lane_list(lane)
    slices = get_slice_list(slice)
    tx_fracn_list =[0]*16
    rx_fracn_list =[0]*16
    frac_mult = 16 if chip_rev(print_en=0) == 2.0 else 1
    
    # Disable Frac-N first before setting them to new values
    for sl in slices:
        sel_slice(sl); 
        for ln in lanes: 
            wreg(0x0d7,0x0000,ln)
            wreg(0x1f0,0x0000,ln)
            wreg(addr=0x0d8,val= 0,lane=ln )            
            wreg(addr=0x1f1,val= 0,lane=ln )
            if tx_ppm==-1: 
                print ("Slice %d Lane %2d: TX and RX FRAC-N Disabled"%(sl,ln))
                
    # Done if disabling Frac-N Only
    if tx_ppm==-1: 
        return
        
    # Calculate New Frac-N values per lane
    for ln in lanes:        
        frac_to_ppm = 2*frac_mult if lane_mode_list[ln] == 'pam4' else 8*frac_mult
        tx_fracn_list[ln] = frac_to_ppm*tx_ppm
        rx_fracn_list[ln] = frac_to_ppm*rx_ppm
        if lane==None:
            tx_fracn_list[ln] += frac_to_ppm*tx_ppm*(ln) if ln<8 else frac_to_ppm*tx_ppm*(ln-8)
            rx_fracn_list[ln] += frac_to_ppm*rx_ppm*(ln) if ln<8 else frac_to_ppm*rx_ppm*(ln-8)
        print ("Lane %2d: TX FRAC-N = %3d, RX FRAC-N = %3d"%(ln, tx_fracn_list[ln], rx_fracn_list[ln]))

    # Program Frac-N divider values before enabling Frac-N
    for sl in slices:
        sel_slice(sl); 
        for ln in lanes:        
            wreg(addr=0x0d8,val= tx_fracn_list[ln],lane=ln )            
            wreg(addr=0x1f1,val= rx_fracn_list[ln],lane=ln )

    # Enable Frac-N with new divider values
    for sl in slices:
        sel_slice(sl); 
        for ln in lanes:        
            wreg(0x0d7,0xa000,ln)
            wreg(0x1f0,0xa000,ln)
    lrr()
    ser()

def top_v():
    sel_slice(0);
    meas_tp(tp_grp=0, tp_num=13, tp_mode="PGATE")
    meas_tp(tp_grp=1, tp_num=13, tp_mode="PGATE")
    sel_slice(1);
    meas_tp(tp_grp=0, tp_num=13, tp_mode="PGATE")
    meas_tp(tp_grp=1, tp_num=13, tp_mode="PGATE")

def hist(hist_time=1, lane='all', slice=[0,1]):
    start_time=time.time()
    result = fec_ana_hist_all(hist_time=hist_time,lane=lane,slice=slice)
    stop_time=time.time()
    #print("\n...Total Histogram time: %2.2f sec"%(stop_time-start_time))
    print("\n\n...Lanes Failed: %d\n"%(result))


def hist_test(off_time=1, hist_time=1,hist_slice=None, loops=1):
    for loop in range(loops):
        print("\n>>>HISTOGRAM TEST LOOP %d"%(loop+1)),
        poff()
        print("...Power OFF Time = %1.1f sec"%(off_time)),
        time.sleep(off_time)
        pon()
        time.sleep(1)
        config_baldeagle(slice=[0,1], mode='pam4',lane='all')
        fec_ana_hist_all(hist_time=hist_time,slice=hist_slice)
        print("\n<<<END of HISTOGRAM TEST LOOP %d"%(loop+1))

def test1(filename = None):
    if filename != None:
        timestr = time.strftime("%Y%m%d_%H%M%S")
        #filename = gChipName + '_Histogram_Test_Log_' + timestr + '.TXT'
        log = open(filename, 'w')
        log.write('\n----------------------------------------------------------------------'),
        log.write('\n Histogram Test Log File: %s' % filename),
        log.write('\n %s Rev %2.1f' % (gChipName, chip_rev(print_en=0))),
        log.write("\n %s" % time.asctime())
        log.write('\n----------------------------------------------------------------------\n\n'),
        log.close()
        log_file = open(filename, 'a+')
        temp_stdout = sys.stdout
        sys.stdout = log_file

    #histo_test(off_time= 1, hist_time=1,hist_slice=0, loops=2)
    histo_test(off_time= 1, hist_time=100,hist_slice=0, loops=20)
    histo_test(off_time=60, hist_time=100,hist_slice=0, loops=20)
    
    if filename != None:
        sys.stdout = temp_stdout
        log_file.close()
        
def rma_test2(set=1):
    if set==0:
        regg(0x9513,0x0840); # TOP-PLL A Div2   
        regg(0x9613,0x0840); # TOP-PLL B Div2
        regg(0x9507,0x0500); # TOP-PLLN A divider
        regg(0x9607,0x0500); # TOP-PLLN B divider
        
        regg(0x0fe,0x116f); # TX lcvcoi = 7
        regg(0x1fd,0x226f); # RX lcvcoi = 7, n=68
        plll(tgt_pll='rx',div2=1,n=68);
        regg(0x0f3,0x0000); # PI OW = 1
        regg(0x1fc,0x1448); # bypass pllpmp=1, bypass1p7=0, bypass1p3=0, bypass1p3_pdet=0
        regg(0x1f4,0x7de6); # bmvco = 1,1111
        regg(0x0da,0x7de6); # bmvco = 1,1111
        
        
        
    elif set==1:
        regg(0x9513,0x0800); # TOP-PLL A Div2   
        regg(0x9613,0x0800); # TOP-PLL B Div2
        regg(0x9507,0x0280); # TOP-PLLN A divider
        regg(0x9607,0x0280); # TOP-PLLN B divider
        #regg(0x0fe,0x1169); # RX lcvcoi = 4, n=34
        #regg(0x1fd,0x1169); # TX lcvcoi = 4
        plll(tgt_pll='rx',div2=0,n=34);
        regg(0x0f3,0x8000); # PI OW = 0
        #regg(0x1fc,0x17c8); # bypass pllpmp=1, bypass1p7=1, bypass1p3_intp_rx=1, bypass1p3_pdet=1
        #regg(0x1f4,0x2de6); # bmvco = 0,1011
        
        
    elif set==2:
        regg(0x9513,0x0800); # TOP-PLL A Div2   
        regg(0x9613,0x0800); # TOP-PLL B Div2
        regg(0x9507,0x0280); # TOP-PLLN A divider
        regg(0x9607,0x0280); # TOP-PLLN B divider
        regg(0x0fe,0x1169); # RX lcvcoi = 4, n=34
        regg(0x1fd,0x1169); # TX lcvcoi = 4
        plll(tgt_pll='rx',div2=0,n=34);
        regg(0x0f3,0x8000); # PI OW = 0
        regg(0x1fc,0x17c8); # bypass pllpmp=1, bypass1p7=1, bypass1p3_intp_rx=1, bypass1p3_pdet=1
        regg(0x1f4,0x2de6); # bmvco = 0,1011
        regg(0x0da,0x2de6); # bmvco = 0,1011
        
        
    elif set==3: # more problems
        regg(0x9513,0x0840); # TOP-PLL A Div2   
        regg(0x9613,0x0840); # TOP-PLL B Div2
        regg(0x9507,0x0500); # TOP-PLLN A divider
        regg(0x9607,0x0500); # TOP-PLLN B divider
        
        regg(0x0fe,0x116f); # TX lcvcoi = 7
        regg(0x1fd,0x226f); # RX lcvcoi = 7, n=68
        plll(tgt_pll='rx',div2=1,n=68);
        regg(0x0f3,0x0000); # PI OW = 1
        regg(0x1fc,0x1448); # bypass pllpmp=1, bypass1p7=0, bypass1p3=0, bypass1p3_pdet=0
        regg(0x1f4,0x7de6); # bmvco = 1,1111
        regg(0x0da,0x7de6); # bmvco = 1,1111
        
        regg(0x9505,0x9c00)
        regg(0x9605,0x9c00)
        regg(0xfc,0x73b6)
        regg(0x1fb,0x49f6)
        
    elif set==4: # rajan
        regg(0x9513,0x0840); # TOP-PLL A Div2   
        regg(0x9613,0x0840); # TOP-PLL B Div2
        regg(0x9507,0x0500); # TOP-PLLN A divider
        regg(0x9607,0x0500); # TOP-PLLN B divider
        
        regg(0x0fe,0x116f); # TX lcvcoi = 7
        regg(0x1fd,0x226f); # RX lcvcoi = 7, n=68
        plll(tgt_pll='rx',div2=1,n=68);
        regg(0x0f3,0x0000); # PI OW = 1
        regg(0x1fc,0x1448); # bypass pllpmp=1, bypass1p7=0, bypass1p3=0, bypass1p3_pdet=0
        regg(0x1f4,0x7de6); # bmvco = 1,1111
        regg(0x0da,0x7de6); # bmvco = 1,1111
        



        
        
        
    lrr();
    rxm(1)
    hist(10)


def Cameo_debug():
    kp(6,1)
    reg(0x1e6,0x88A5)
    reg(0x1fd,0x2269)
    reg(0x1f4,0x2de6)
    reg(0x1fb,0x49c6)
    reg(0x1db,0x6fb6)


def mdio_test(loop_times):
    break_flag = False
    for i in range(loop_times+1):
        for j in range(10):
            data = random.randint(0, 0xffff)
            addr = 0x9f00 + j
            MdioWr(addr, data)
            readout = MdioRd(addr)
            if readout != data:
                print "addr: %04x, readout: %04x, data: %04x" % (addr, readout, data)
                break_flag = True
                break
        if break_flag:
            break
        if i % 100 == 0:
			print "%d" % i
def cameo_lscpcie_open():
    lock1.acquire()
    try:
        libcameo.lscpcie_open() 
        print "cameo_lscpcie_open() in lock"
        time.sleep(1) 
    finally:        
        lock1.release()
        
    
def cameo_sw_phy_card(phy,card):
    lock2.acquire()
    try:
        libcameo.cm_sw_phy_card(phy,card) 
        time.sleep(1) 
    finally:        
        lock2.release()
    
def Cameo_credo100G(card,gearbox_type='100G-1'):
    print ("Cameo_credo100G card %d " % card) 
    #time.sleep(20)        
    #return True
    gUsbPort=99
    gSlice=0
    slices=[0,1]
    cameo_lscpcie_open()      
    all_start = time.time()

    cameo_sw_phy_card(1,card) 
    silicon_revision = chip_rev(print_en=1)
    
    #gFwFileName=fw_path
    if silicon_revision == 2.0:
        gFwFileName = fw_path + '/BE2.fw.2.15.04.bin'
    elif silicon_revision == 1.0:
        gFwFileName = fw_path + '/BE.fw.2.15.04.bin'
    else:
        print "Failed to access silicon_revision"
        return False    
        
        
    for num in range(1,5):
        cameo_sw_phy_card(num,card)
        mdio(connect='connect', Slice=0, usb_port=0)
        mdio_status = get_mdio_status()

        if (mdio_status == MDIO_CONNECTED):
            soft_reset()
            if fw_load(gFwFileName,sl=[0,1]) == False:
                print "Failed to load FW %s (Silicon rev:%f)" %(gFwFileName,silicon_revision)
                return False
            #sel_slice(0)
            #fw_load_broadcast_mode('ON')
            #sel_slice(1)
            #fw_load_broadcast_mode('ON')
        else:
            print "Failed to access MDIO"
            return False
    #if fw_load(gFwFileName,sl=[0,1]) == False:
    #    return False
    #for num in range(1,5):
    #    cameo_sw_phy_card(num,card)
    #    mdio(connect='connect', Slice=0, usb_port=0)
    #    mdio_status = get_mdio_status()
    #
    #    if (mdio_status == MDIO_CONNECTED):
    #        sel_slice(0)
    #        fw_load_broadcast_mode('OFF')
    #        sel_slice(1)
    #        fw_load_broadcast_mode('OFF')
    #    else:
    #        print "Failed to access MDIO"    
    for num in range(1,5):
        cameo_sw_phy_card(num,card)
        mdio(connect='connect', Slice=0, usb_port=0)
        mdio_status = get_mdio_status()

        if (mdio_status == MDIO_CONNECTED):
            sel_slice(0)
            if not fw_loaded(print_en=0):
                print("\n*** No FW Loaded. Skipping fw_pause() \n")
                return False
            fw_config_lane (mode='pam4', lane=[0,1])
            fw_config_lane (mode='nrz', lane=[8,9,10,11])
            slice=0
            A_lanes=[0,1,4,5]
            gearbox_by_fw=True
            fec_b_bypass=False
            config_baldeagle_gearbox(slice, A_lanes,gearbox_type, gearbox_by_fw, fec_b_bypass)
            tx_taps(tap1=0,tap2=0,tap3=20,tap4=8,tap5=0,lane=0)
            tx_taps(tap1=0,tap2=0,tap3=20,tap4=8,tap5=0,lane=1)  
            tx_taps(tap1=0,tap2=0,tap3=0,tap4=0,tap5=0,lane=2)
            tx_taps(tap1=0,tap2=0,tap3=0,tap4=0,tap5=0,lane=3)
            tx_taps(tap1=0,tap2=0,tap3=20,tap4=8,tap5=0,lane=4)
            tx_taps(tap1=0,tap2=0,tap3=20,tap4=8,tap5=0,lane=5)
            tx_taps(tap1=0,tap2=0,tap3=0,tap4=0,tap5=0,lane=6)
            tx_taps(tap1=0,tap2=0,tap3=0,tap4=0,tap5=0,lane=7)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=8)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=9)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=10)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=11)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=12)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=13)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=14)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=15)
            
            sel_slice(1)
            fw_config_lane(mode='pam4', lane=[0,1])
            fw_config_lane(mode='nrz', lane=[8,9,10,11])
            slice=1
            A_lanes=[0,1,4,5]
            gearbox_by_fw=True
            fec_b_bypass=False            
            config_baldeagle_gearbox(slice, A_lanes,gearbox_type, gearbox_by_fw, fec_b_bypass)
            tx_taps(tap1=0,tap2=0,tap3=20,tap4=8,tap5=0,lane=0)
            tx_taps(tap1=0,tap2=0,tap3=20,tap4=8,tap5=0,lane=1)            
            tx_taps(tap1=0,tap2=0,tap3=0,tap4=0,tap5=0,lane=2)
            tx_taps(tap1=0,tap2=0,tap3=0,tap4=0,tap5=0,lane=3)
            tx_taps(tap1=0,tap2=0,tap3=20,tap4=8,tap5=0,lane=4)
            tx_taps(tap1=0,tap2=0,tap3=20,tap4=8,tap5=0,lane=5)
            tx_taps(tap1=0,tap2=0,tap3=0,tap4=0,tap5=0,lane=6)
            tx_taps(tap1=0,tap2=0,tap3=0,tap4=0,tap5=0,lane=7)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=8)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=9)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=10)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=11)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=12)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=13)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=14)
            tx_taps(tap1=0,tap2=-8,tap3=17,tap4=0,tap5=0,lane=15)

        else:
            print "Failed to access MDIO"
            return False


    libcameo.lscpcie_close()
    all_end = time.time()
    all_elapsed = all_end - all_start
    print "\nCard: %d" %card
    print "All Time taken: ", all_elapsed, "seconds."  
    return True    
#################################################################################################### 
#################################################################################################### 
# 
#                  END of all Bald Eagle Scripts
#
# Every time executing this file:
# Call the following functions first to establish basic connection 
#                or to define certain global reference parameters
#   
####################################################################################################
####################################################################################################
if __name__ == "__main__":
    card=int(sys.argv[1])
    Cameo_credo100G(card)    

       
    
########################################################################################################################################    
########################################################################################################################################    
   
