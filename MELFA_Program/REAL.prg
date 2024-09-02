1 '***********************************
2 ' ADIRO Realtime program
3 '***********************************
4 '-----------------------------------
5 '| © ADIRO Automatisierungstechnik GmbH, 73734 Esslingen, 2024
6 '|
7 '| The reproduction, distribution and utilisation of this document, as well as the communication of its
8 '| contents to others without explicit authorisation, is prohibited. Offenders will be held liable for
9 '| compensation of damages. All rights reserved, in particular the right to file patent, utility model and
10 '| registered design applications.
11 '-----------------------------------
12 '
13 '-----------------------------------
14 ' Declaration of local Positions
15 '-----------------------------------
16 Def Jnt JStart
17 '
18 '-----------------------------------
19 ' Application main entry point
20 '-----------------------------------
21 Function V main()
22   Servo On
23   JOvrd 10
24     JStart=SetJnt(+0.0,-0.0,+1.5708,-0.0,+1.5708,+0.0)
25   Mov JStart
26   Dly 0.2
27   JOvrd 100
28   Open "ENET:192.168.3.58" As #1
29   M_Out(2512)=1
30   Mxt 1,1
31   Hlt
32   Exit Function
33 FEnd
34 '
35 End
