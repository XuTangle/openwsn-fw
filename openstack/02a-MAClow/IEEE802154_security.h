/**
\brief Security operations defined by IEEE802.15.4 standard

\author Savio Sciancalepore <savio.sciancalepore@poliba.it>, April 2015.
\author Giuseppe Piro <giuseppe.piro@poliba.it>,
\author Gennaro Boggia <gennaro.boggia@poliba.it>,
\author Luigi Alfredo Grieco <alfredo.grieco@poliba.it>
\author Tengfei Chang <tengfei.chang@eecs.berkeley.edu>, June 2015.
*/

#ifndef __IEEE802154_SECURITY_H
#define __IEEE802154_SECURITY_H

/**
\addtogroup helpers
\{
\addtogroup IEEE802154
\{
*/

#include "opendefs.h"
#include "packetfunctions.h"
#include "neighbors.h"

//=========================== define ==========================================

#define L2_SECURITY_ACTIVE

#ifdef L2_SECURITY_ACTIVE
// TODO use parameters passed by SCons
    #define IEEE802154E_SECURITY_LEVEL           ASH_SLF_TYPE_CRYPTO_MIC32
    #define IEEE802154E_SECURITY_LEVEL_BEACON    ASH_SLF_TYPE_MIC_32
    #define IEEE802154E_SECURITY_KEYIDMODE       ASH_KEYIDMODE_DEFAULTKEYSOURCE
    #define IEEE802154E_SECURITY_KEY_INDEX       1
#else
    #define IEEE802154E_SECURITY_LEVEL           ASH_SLF_TYPE_NOSEC
    #define IEEE802154E_SECURITY_LEVEL_BEACON    ASH_SLF_TYPE_NOSEC 
    #define IEEE802154E_SECURITY_KEYIDMODE       0
    #define IEEE802154E_SECURITY_KEY_INDEX       0
#endif // L2_SECURITY_ACTIVE

#define MAXNUMKEYS           MAXNUMNEIGHBORS+1

enum Auxiliary_Security_Header_scf_enums{ //Security Control Field
   ASH_SCF_SECURITY_LEVEL = 0,
   ASH_SCF_KEY_IDENTIFIER_MODE = 3,
   ASH_SCF_FRAME_CNT_MODE = 5,
   ASH_SCF_FRAME_CNT_SIZE = 6,
};

enum Auxiliary_Security_Header_slf_enums{ //Security Level Field
   ASH_SLF_TYPE_NOSEC = 0,
   ASH_SLF_TYPE_MIC_32 = 1,
   ASH_SLF_TYPE_MIC_64 = 2,
   ASH_SLF_TYPE_MIC_128 = 3,
   ASH_SLF_TYPE_ONLYCRYPTO = 4,
   ASH_SLF_TYPE_CRYPTO_MIC32 = 5,
   ASH_SLF_TYPE_CRYPTO_MIC64= 6,
   ASH_SLF_TYPE_CRYPTO_MIC128 = 7,
};

enum Auxiliary_Security_Header_keyIdMode_enums{ //Key Identifier Mode Field
   ASH_KEYIDMODE_IMPLICIT = 0,
   ASH_KEYIDMODE_DEFAULTKEYSOURCE = 1,
   ASH_KEYIDMODE_EXPLICIT_16 = 2,
   ASH_KEYIDMODE_EXPLICIT_64 = 3,
};

//=========================== typedef =========================================

typedef struct{//identifier of the device which is using the key
	open_addr_t       deviceAddress;
	macFrameCounter_t FrameCounter;
	bool              Exempt;
} m_deviceDescriptor;

typedef struct{//descriptor of the key we are looking for
   uint8_t      KeyIdMode;
   uint8_t      KeyIndex;
   open_addr_t  KeySource;
   open_addr_t  PANId;
   open_addr_t  Address;
} m_keyIdLookupDescriptor;

typedef struct{//descriptor of the Security Level for this type of communication
   uint8_t FrameType;
   uint8_t CommandFrameIdentifier; //if the FrameType is a command, this specify what kind of command is
   uint8_t SecurityMinimum; //minimum required
   bool    DeviceOverrideSecurityMinimum; //if true, this indicate that the minimum can be overridden
   uint8_t AllowedSecurityLevels[7]; //set of Security Levels Allowed for incoming MAC frames
} m_securityLevelDescriptor;

typedef struct{//defines what kind of frame the key is intended
   uint8_t FrameType;
   uint8_t CommandFrameIdentifier;
} m_keyUsageDescriptor;

typedef struct{//Table which contains the device that are currently using this key
   m_deviceDescriptor DeviceDescriptorEntry[MAXNUMNEIGHBORS-1];
} m_macDeviceTable;


typedef struct{//descriptor of the key
   m_keyIdLookupDescriptor KeyIdLookupList;
   m_macDeviceTable*       DeviceTable;
   m_keyUsageDescriptor    KeyUsageList[3];
   uint8_t                 key[16];
} m_keyDescriptor;

typedef struct{
   m_keyDescriptor KeyDescriptorElement[MAXNUMKEYS-1];
} m_macKeyTable;

typedef struct{
   m_securityLevelDescriptor SecurityDescriptorEntry[5];
} m_macSecurityLevelTable;


//=========================== variables =======================================
typedef struct{
   macFrameCounter_t       m_macFrameCounter;
   uint8_t                 m_macFrameCounterMode;
   uint8_t                 m_macAutoRequestKeyIdMode;
   uint8_t                 m_macAutoRequestSecurityLevel;
   uint8_t                 m_macAutoReququestKeyIndex;
   open_addr_t             m_macDefaultKeySource;
   m_macKeyTable           MacKeyTable;
   m_macDeviceTable        MacDeviceTable;
   m_macSecurityLevelTable MacSecurityLevelTable;
   uint8_t                 M_k[16];
}ieee802154_security_vars_t;

//=========================== prototypes ======================================

//admin
void      IEEE802154_security_init(void);
//public
void      IEEE802154_security_prependAuxiliarySecurityHeader(OpenQueueEntry_t* msg);
owerror_t IEEE802154_security_outgoingFrameSecurity(OpenQueueEntry_t* msg);
void      IEEE802154_security_retrieveAuxiliarySecurityHeader(
                            OpenQueueEntry_t*      msg,
                            ieee802154_header_iht* tempheader
);
owerror_t IEEE802154_security_incomingFrame(OpenQueueEntry_t* msg);

/**
\}
\}
*/

#endif
