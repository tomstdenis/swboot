- (U1) 1 2x12 pin Pro Micro (atmega32u4 based) board (USB port to the edge)
- 2 10k Ohm 1206 SMD resistors (1/4W -> 1/2W)
- 2 100 Ohm 1206 SMD resistors (1/4W -> 1/2W)
- 1 500-1000 Ohm 1206 SMD resistor (1/4W -> 1/2W) (I used a 680 in my prototype...)
- (C1) 1 470uF SMD capacitor (16V (*)) (I used the CAP-SMD_BD6.3-L6.6-W6.6-FD footprint from EasyEDA)
- (H1) 1 1x4 male pin header (2.54mm spacing)
- (Q1, Q2, Q3) 3 DMG2301L-7 SOT-3 P Channel mosfets

The resistors aren't named properly but the silkscreen clearly labels what goes where.

(*) This is a 5V system but we want to ensure good headroom for capacitance and any minor voltage deviations.
