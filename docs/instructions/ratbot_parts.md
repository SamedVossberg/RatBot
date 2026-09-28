# RatBot Parts

[Sourcing Components](sourcing_components.md)

[Assembling the Robot](robot_assembly.md)

[Software Setup](software_setup.md)

[**RatBot Parts**]()

[Back to Project Page](https://github.com/SamedVossberg/RatBot)

All numbers below come from the Fusion design `Q8bot_rev2_5_SQuRo_Leg`, version 30, which the STL files were exported from. Masses use the CAD materials: PA12 for the printed parts, 18 g per servo and 25 g per cell. The complete robot comes to 297 g as modelled.

## Printed Parts

Download `RatBot_hw-v1.0.zip` from the [Releases](https://github.com/SamedVossberg/RatBot/releases) page. One robot takes 29 prints from 18 files, 46.3 g in total. The parts are modelled in HP 3D HR CB PA 12 for MJF printing (HP Jet Fusion 580 Color).

`_P` and `_N` files are mirror images of each other. Seen from behind, with the head pointing away from you, the `_P` parts go on the right side (+Z in the CAD) and the `_N` parts on the left (−Z). All four limbs use the same five links; the fore and hind limbs differ only in the ulna.

| STL file | Print | Mass each | Size (mm) | Part |
|---|:---:|---:|---|---|
| `RatBot_Main_Frame_HeadMount_Belly.stl` | 2 | 6.76 g | 144.8 × 37.6 × 15.1 | Frame half: the Q8bot frame with mounting tongues on the belly side for the neck clamp and the tail |
| `RatBot_SQ3_ShoulderCrank_P.stl` | 2 | 1.52 g | 21.5 × 23.9 × 9.0 | Limb: shoulder crank |
| `RatBot_SQ3_ShoulderCrank_N.stl` | 2 | 1.52 g | 21.5 × 23.9 × 9.0 | Limb: shoulder crank |
| `RatBot_SQ3_ElbowCrank_P.stl` | 2 | 1.02 g | 20.8 × 23.7 × 4.5 | Limb: elbow crank |
| `RatBot_SQ3_ElbowCrank_N.stl` | 2 | 1.02 g | 20.8 × 23.7 × 4.5 | Limb: elbow crank |
| `RatBot_SQ3_Humerus_P.stl` | 2 | 0.79 g | 29.1 × 34.3 × 4.5 | Limb: humerus |
| `RatBot_SQ3_Humerus_N.stl` | 2 | 0.79 g | 29.1 × 34.3 × 4.5 | Limb: humerus |
| `RatBot_SQ3_KneePushrod_P.stl` | 2 | 0.73 g | 30.4 × 35.5 × 4.0 | Limb: knee pushrod |
| `RatBot_SQ3_KneePushrod_N.stl` | 2 | 0.73 g | 30.4 × 35.5 × 4.0 | Limb: knee pushrod |
| `RatBot_SQ3_HipPushrod_P.stl` | 2 | 0.41 g | 27.9 × 8.4 × 4.0 | Limb: hip pushrod |
| `RatBot_SQ3_HipPushrod_N.stl` | 2 | 0.41 g | 27.9 × 8.4 × 4.0 | Limb: hip pushrod |
| `RatBot_SQ3_UlnaFoot_P.stl` | 1 | 0.86 g | 41.3 × 41.3 × 4.5 | Forelimb: ulna with foot |
| `RatBot_SQ3_UlnaFoot_N.stl` | 1 | 0.86 g | 41.3 × 41.3 × 4.5 | Forelimb: ulna with foot |
| `RatBot_SQ4_UlnaFootHind_P.stl` | 1 | 1.29 g | 66.1 × 48.1 × 4.5 | Hindlimb: ulna with foot |
| `RatBot_SQ4_UlnaFootHind_N.stl` | 1 | 1.29 g | 66.1 × 48.1 × 4.5 | Hindlimb: ulna with foot |
| `RatBot_RatHead_Cap_v2.stl` | 1 | 5.04 g | 47.2 × 30.2 × 34.8 | Head |
| `RatBot_RatHead_Neck_Clamp.stl` | 1 | 1.46 g | 23.8 × 9.0 × 15.8 | Neck clamp, slips over the frame tongue at the front |
| `RatBot_RatTail.stl` | 1 | 4.11 g | 158.5 × 30.8 × 15.8 | Passive tail with integral yoke |

Each STL keeps its part coordinates from the CAD, so a flat link can sit a few millimetres above z = 0. Slicers and print services drop it onto the bed.

## Off-the-Shelf Parts

| Part | Qty | Notes |
|---|:---:|---|
| DYNAMIXEL XL330-M077-T | 9 | Eight for the legs (IDs 11 to 18) and one for the head |
| 692ZZ ball bearing, 2 × 6 × 3 mm | 24 | Six per limb |
| M2 self-tapping screw, 6 mm | 42 | |
| M2 self-tapping screw, 8 mm | 6 | |
| M2 socket head cap screw, head assembly | 3 | Ø3.8 mm heads, modelled 19.3 mm and 2 × 9.3 mm long including the head |
| M2 × 18 socket head cap screw with M2 nut | 1 | Holds the tail yoke on the frame tabs |
| Q8bot main PCB | 1 | Unchanged Q8bot board (rev 2.3 in the CAD). Gerbers and BOM are in the [Q8bot releases](https://github.com/EricYufengWu/q8bot/releases) |
| 14500 Li-ion cell, KeepPower P1450C2 | 2 | See [Sourcing Components](sourcing_components.md) |
| Keystone battery clips | 2 + 2 | `PN1017-1` and `PN1087-1` in the CAD, the same clips as Q8bot |
| Seeed Studio XIAO ESP32C3 | 1 | Controller dongle for the laptop, not part of the CAD |
