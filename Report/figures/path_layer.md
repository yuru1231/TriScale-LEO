## sat2sat
Source: SAT src
Endpoint stack: App -> NetDevice -> MAC -> PHY

傳送路徑:
SAT src -> ISL transit -> SAT dst

主要傳送層:
ISL / NetDevice / Link layer

觀測點:
ISL drop trace / ISL NetDevice Tx/Rx
可看 PacketDropRateTrace, scoped ISL rx/drop counters

Final verdict:
ISL_LAYER
pass if scoped ISL link exists and rxPkts > 0

物理節點:
SAT src / SAT dst 都是 physical satellite node

## sat2gw
Source: SAT src
Destination: GW dst

傳送路徑:
SAT -> return feeder -> GW physical receive -> GW user/application endpoint

主要傳送層:
Feeder link PHY/MAC
Endpoint App layer only for final delivery support

觀測點:
OrbiterRxFeeder / GW-side feeder Rx counter
GW user PacketSink::Rx if endpoint app is installed

Final verdict:
FEEDER_LAYER
pass if scoped feeder rxPkts > 0

物理節點:
SAT = physical satellite node
GW physical node must be checked with GetGwNodes()
GW user node is endpoint/user-side node, not necessarily the physical GW radio node

## gw2sat
Source: GW user / GW physical node
Destination: SAT

傳送路徑:
GW user -> physical GW -> feeder uplink -> SAT

主要傳送層:
Feeder link PHY/MAC

觀測點:
GW feeder Tx/Rx or satellite feeder receive trace
scope usually: gwtx<gwId> -> sat<entry>

Final verdict:
FEEDER_LAYER
pass if feeder uplink/return feeder observation has rxPkts > 0

物理節點:
GW logical ID must be mapped to physical GW node
SAT entry is physical satellite node

## sat2ut
Source: SAT src
Destination: UT dst

傳送路徑:
SAT -> service downlink -> UT physical receive -> UT user/application endpoint

主要傳送層:
Service link PHY/MAC

觀測點:
Service link Rx trace
UT-side receive trace
PacketSink::Rx only as endpoint confirmation

Final verdict:
SERVICE_LAYER
main rule: service rxPkts > 0
endpoint probe is diagnostic, not a separate verdict layer

物理節點:
SAT = physical satellite node
UT node = physical/user terminal node
UT user node = endpoint/application-side node

## gw2ut_e2e
Source: GW user
Destination: UT user

傳送路徑:
GW user -> physical GW
-> feeder uplink GW -> SAT
-> optional ISL transit SAT -> SAT
-> service downlink SAT -> UT
-> UT user endpoint

主要傳送層:
Feeder PHY/MAC + optional ISL Link/NetDevice + Service PHY/MAC + App endpoint

觀測點:
Feeder observe: GW/SAT feeder counters
ISL observe: ISL NetDevice drop/rx counters
Service observe: service link Rx
Endpoint observe: UT PacketSink::Rx

Final verdict:
Custom mapping:
FEEDER_LAYER + SERVICE_LAYER
and ISL_LAYER only when route has ISL hop

物理節點:
GW physical node: must be confirmed
SAT entry/exit/serving: physical satellite nodes
UT node: physical UT node
UT user: endpoint node

## gw2gw_e2e achieved

Source:
GW user src

Destination:
GW user dst

傳送路徑:
GW user src
-> physical GW src
-> feeder uplink: GW src -> entry SAT
-> optional ISL route: entry SAT -> ... -> exit SAT
-> feeder downlink: exit SAT -> physical GW dst
-> GW user dst

主要傳送層:
Feeder link + optional ISL link + feeder link

若 route 有 ISL hop:
main transmission layers = FEEDER_LAYER + ISL_LAYER + FEEDER_LAYER

若 entry SAT 與 exit SAT 相同:
main transmission layers = FEEDER_LAYER only, no ISL hop

觀測點:
PacketSink::Rx at destination GW user
Feeder uplink Rx at entry SAT
Feeder downlink Rx at physical GW dst
ISL drop/load/rx counters if ISL path exists
Physical GW inventory via GetGwNodes()
GW user endpoints via GetGwUserNode()

Final verdict:
Application-layer PASS if destination GW user PacketSink::Rx > 0

Physical validation:
PASS only if both physical GW nodes exist:
physical GW src = confirmed
physical GW dst = confirmed

Link-layer validation:
FEEDER_LAYER PASS if feeder uplink/downlink rxPkts > 0
ISL_LAYER PASS if route has ISL hop and scoped ISL rxPkts > 0
If no ISL hop, ISL_LAYER is N/A, not FAIL

物理節點:
GW user src/dst are endpoint/application nodes
physical GW src/dst are real ns-3 GW nodes
SAT entry/exit/transit are real satellite nodes
Logical gw0/gw1/gw2 must map to confirmed physical GW nodes before being drawn as physical

圖上可標:
gw2gw achieved = application delivery + physical GW src/dst confirmed + required link observations present