X25519 is only for key exchange.
For signatures you need Ed25519 which is not yet implemented.

Curve25519
y^2 = x^3 + 486662x^2 + x
y^2 = x^3 + ax + b

MAC: 86 8A 46 0C 00 4B 12 00 //hexraw_print(8, ((uint8_t*)(0x500012F0)));


secp256k1 : SECG curve over a 256 bit prime field //nistp256

private:

public:
046c74efd5ee2e0f4eee3999635bd4c5c852a31a47075541b60729ac29c43dfb982b8ce23a14f99dafa6ff8e1577eec232eb61c9436c7bfa3c15818e7c360fccda
04 6c 74 ef d5 ee 2e 0f 4e ee 39 99 63 5b d4 c5 c8 52 a3 1a 47 07 55 41 b6 07 29 ac 29 c4 3d fb 98 2b 8c e2 3a 14 f9 9d af a6 ff 8e 15 77 ee c2 32 eb 61 c9 43 6c 7b fa 3c 15 81 8e 7c 36 0f cc da











Для описания кривой в стандарте NIST используется набор из 6 параметров D=(p,a,b,G,n,h), где 

p — простое число, модуль эллиптической кривой;
a, b — задают уравнение эллиптической кривой image;
G — точка эллиптической кривой большого порядка. Это означает что если умножать точку на числа меньшие, чем порядок точки, каждый раз будут получаться совершенно различные точки;
n — порядок точки G;
h — параметр, называемый кофактор. Определяется отношением общего числа точек на эллиптической кривой к порядку точки G. Данное число должно быть как можно меньше.


MAC: 86 8A 46 0C 00 4B 12 00 86 8A 46 0C 00 4B 12 00 86 8A 46 0C 00 4B 12 00 86 8A 46 0C 00 4B 12 00
NUM_ECC_DIGITS: 8
sizeof EccPoint: 64
EccPoint: 
602A0020B59A00000C000000F017010000000940478C0000184A00100100000001000000195300008831002058090020100500207953000071530000D14F0000

l_private: 
D03900207731002078310020583D00200000000013FF00002800000060090020

l_hash: 
600900203405002034050020B9510000E00D0020280500201318010013180100

l_random: 
05220010F9FFFFFF5B2A00205B2A002000000000F9FFFFFF35390020592A0020

r: 
000000000000000002124B00D38A0000BE0D0100000200810000000038390020

s: 
4A390020120000004C2A0020383900204A39002012000000304F0020304F0020


EccPoint: 602A0020399B00000C0000004818010000000940CB8C0000184A001001000000010000009D530000883100205809002010050020FD530000F553000055500000
l_private: D03900207731002078310020583D002000000000BFFF000028000000C0390020
l_hash: C43900203B9500007400000044060020C0390020B59A00006B1801006B180100
l_random: 1E000000F9FFFFFF0200000001000000070000001E00000000000000855B0000
r: 2A220010001400254C2A00204C2A0020602A0020000000010000000038390020
s: 4A390020120000004C2A0020383900204A39002012000000304F0020304F0020
Testing
ecc_make_key() ok
EccPoint: 533DF9E868A85A91E9FACDDAC24831EC00311D3974EBD241E40D3656CB8F7B9EDA16BD93DABEF54A3576CD9DFCE99EC442C5BDB064DE739F3994025CE8A74B94
l_private: D03900207731002078310020583D002000000000BFFF000028000000C0390020
ecdsa_sign() ok
l_private: D03900207731002078310020583D002000000000BFFF000028000000C0390020
l_hash: C43900203B9500007400000044060020C0390020B59A00006B1801006B180100
l_random: 1E000000F9FFFFFF0200000001000000070000001E00000000000000855B0000
r: 47460B01ACF6F1123DB43BE614874183427732F991D50295074967AC7A222723
s: 8C5FDB89CB035D779116AB7095405B6E3C251806403FB6881D42E34A79BB9E66
Valid public key!
ecdsa_verify() ok

533DF9E868A85A91E9FACDDAC24831EC00311D3974EBD241E40D3656CB8F7B9E
DA16BD93DABEF54A3576CD9DFCE99EC442C5BDB064DE739F3994025CE8A74B94



EccPoint: 50050DD9A74779933E3FA1C534F8ADCAD3DCE7B43D2E51C713B6E1DC790414E7111E3BAC726B20FDAF3CEBF991FB58F14A18F0686B85DC58BC2ACE54ACB47E89
Not a valid public key!
EccPoint: 
3322110077665544110099880200000003000000040000000500000006000000
2233001166774455110099880200009003000080040000700500006006000050

//d = 24 B8 50 94 0A 9B 79 02 14 D7 7E 95 D4 0B 11 49 5A 30 D5 80 CF 8C 39 0B BC 09 F1 BC FA 10 88 06
0x24B85094 
0x0A9B7902 
0x14D77E95 
0xD40B1149
0x5A30D580
0xCF8C390B
0xBC09F1BC
0xFA108806

l_private[0] = 0xFA108806;
l_private[1] = 0xBC09F1BC;
l_private[2] = 0xCF8C390B;
l_private[3] = 0x5A30D580;
l_private[4] = 0xD40B1149;
l_private[5] = 0x14D77E95;
l_private[6] = 0x0A9B7902;
l_private[7] = 0x24B85094;


//x = 6F F5 1D 33 18 D0 D2 DC 6D 54 D4 42 3E 5E 13 34 F3 69 8B 3E 99 03 48 67 A4 77 19 F1 2F DE BE 6C
0x6FF51D33 
0x18D0D2DC
0x6D54D442
0x3E5E1334
0xF3698B3E
0x99034867
0xA47719F1
0x2FDEBE6C

l_public.x[0] = 0x2FDEBE6C;
l_public.x[1] = 0xA47719F1;
l_public.x[2] = 0x99034867;
l_public.x[3] = 0xF3698B3E;
l_public.x[4] = 0x3E5E1334;
l_public.x[5] = 0x6D54D442;
l_public.x[6] = 0x18D0D2DC;
l_public.x[7] = 0x6FF51D33;


//y = AE F2 B7 CD 47 37 41 83 9B 96 F6 A5 A4 D8 C0 8B CC 76 9E 3E 14 12 9C C5 06 FC C1 EB 0C 36 BF 5C
0xAEF2B7CD
0x47374183
0x9B96F6A5
0xA4D8C08B
0xCC769E3E
0x14129CC5
0x06FCC1EB
0x0C36BF5C

l_public.y[0] = 0x0C36BF5C;
l_public.y[1] = 0x06FCC1EB;
l_public.y[2] = 0x14129CC5;
l_public.y[3] = 0xCC769E3E;
l_public.y[4] = 0xA4D8C08B;
l_public.y[5] = 0x9B96F6A5;
l_public.y[6] = 0x47374183;
l_public.y[7] = 0xAEF2B7CD;


//hash = 15 7D 3F 09 0D 8A 09 10 C3 C1 A6 8C E4 10 21 BD 0E A4 15 C5 DF A8 C1 5C 3C D3 07 79 F5 BE 64 2A
0x157D3F09
0x0D8A0910
0xC3C1A68C
0xE41021BD
0x0EA415C5
0xDFA8C15C
0x3CD30779
0xF5BE642A

l_hash[0] = 0xF5BE642A;
l_hash[1] = 0x3CD30779;
l_hash[2] = 0xDFA8C15C;
l_hash[3] = 0x0EA415C5;
l_hash[4] = 0xE41021BD;
l_hash[5] = 0xC3C1A68C;
l_hash[6] = 0x0D8A0910;
l_hash[7] = 0x157D3F09;


//random = 48 7A 0D 38 75 BD B8 12 9A 13 3F 00 5B 81 EC 32 F6 FB A3 06 5B C9 EE 44 06 0B 6E D5 73 53 83 2C
0x487A0D38
0x75BDB812
0x9A133F00
0x5B81EC32
0xF6FBA306
0x5BC9EE44
0x060B6ED5
0x7353832C

l_random[0] = 0x7353832C;
l_random[1] = 0x060B6ED5;
l_random[2] = 0x5BC9EE44;
l_random[3] = 0xF6FBA306;
l_random[4] = 0x5B81EC32;
l_random[5] = 0x9A133F00;
l_random[6] = 0x75BDB812;
l_random[7] = 0x487A0D38;

//r = 87 AE C4 33 E5 AC 57 32 69 E3 48 09 F3 6E D7 2D 2A DC 71 5B 1A 79 6B 4B 47 0A AC 17 54 4D 75 96
//r: 96754D5417AC0A474B6B791A5B71DC2A2DD76EF30948E3693257ACE533C4AE87
0x87AEC433
0xE5AC5732
0x69E34809
0xF36ED72D
0x2ADC715B
0x1A796B4B
0x470AAC17
0x544D7596

r[0] = 0x544D7596;
r[1] = 0x470AAC17;
r[2] = 0x1A796B4B;
r[3] = 0x2ADC715B;
r[4] = 0xF36ED72D;
r[5] = 0x69E34809;
r[6] = 0xE5AC5732;
r[7] = 0x87AEC433;


//s = A9 01 13 11 BF 3E 84 9D 72 A4 77 82 E5 BC 34 3F 22 36 19 3D AB D9 D1 00 93 91 B4 5C 11 A9 A5 0E
s: 0EA5A9115CB4919300D1D9AB3D1936223F34BCE58277A4729D843EBF111301A9
0xA9011311
0xBF3E849D
0x72A47782
0xE5BC343F
0x2236193D
0xABD9D100
0x9391B45C
0x11A9A50E

s[0] = 0x11A9A50E;
s[1] = 0x9391B45C;
s[2] = 0xABD9D100;
s[3] = 0x2236193D;
s[4] = 0xE5BC343F;
s[5] = 0x72A47782;
s[6] = 0xBF3E849D;
s[7] = 0xA9011311;


//VALID
EccPoint: 6CBEDE2FF11977A4674803993E8B69F334135E3E42D4546DDCD2D018331DF56F5CBF360CEBC1FC06C59C12143E9E76CC8BC0D8A4A5F6969B83413747CDB7F2AE

l_public.x[0] = 0x2FDEBE6C;
l_public.x[1] = 0xA47719F1;
l_public.x[2] = 0x99034867;
l_public.x[3] = 0xF3698B3E;
l_public.x[4] = 0x3E5E1334;
l_public.x[5] = 0x6D54D442;
l_public.x[6] = 0x18D0D2DC;
l_public.x[7] = 0x6FF51D33;

l_public.y[0] = 0x0C36BF5C;
l_public.y[1] = 0x06FCC1EB;
l_public.y[2] = 0x14129CC5;
l_public.y[3] = 0xCC769E3E;
l_public.y[4] = 0xA4D8C08B;
l_public.y[5] = 0x9B96F6A5;
l_public.y[6] = 0x47374183;
l_public.y[7] = 0xAEF2B7CD;



миккроконтроллер:
Primary IEEE Address
x
y

hash
r
s

1. Запускается загрузчик.
2. 


комп:
Primary IEEE Address
d

hash
k






0a 0a 2f 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d  ../-------------
2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d  ----------------
2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d  ----------------
2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2f 0a  --------------/.
d0 39 00 20 77 31 00 20 78 31 00 20 58 3d 00 20  Р9. w1. x1. X=. 
00 00 00 00 17 01 01 00 28 00 00 00 c0 39 00 20  ........(...А9. 
c4 39 00 20 93 96 00 00 74 00 00 00 44 06 00 20  Д9. “–..t...D.. 
c0 39 00 20 0d 9c 00 00 d0 39 00 20 00 00 00 00  А9. .њ..Р9. ....
28 00 00 00 74 00 00 00 00 00 00 00 a1 9c 00 00  (...t.......Ўњ..
e0 0d 00 20 38 06 00 20 02 00 00 00 59 80 00 00  а.. 8.. ....YЂ..
e0 0d 00 20 08 07 00 20 08 31 00 20 51 82 00 00  а.. ... .1. Q‚..
00 00 00 00 f0 30 00 20 00 00 00 00 31 9b 00 00  ....р0. ....1›..
0a 00 00 00 00 60 09 00 20 60 09 00 20 34 05 00  .....`.. `.. 4..
20 34 05 00 20 bd 53 00 00 e0 0d 00 20 00 09 00   4.. ЅS..а.. ...
20 00 09 00 20 54 31 00 20 54 31 00 20 0f 31 00   ... T1. T1. .1.
00 cc 05 00 20 68 3f 00 20 68 3f 00 20 db 7d 00  .М.. h?. h?. Ы}.
00 e0 0d 00 20 cc 05 00 20 82 00 00 00 a5 7d 00  .а.. М.. ‚...Ґ}.
00 0c 00 00 00 f0 18 01 00 00 00 09 40 59 80 00  .....р......@YЂ.
00 03 05 00 20 00 10 00 20 0c 8b 01 10 30 8b 01  .... ... .‹..0‹.
10 54 8b 01 10 01 00 00 00 74 31 00 20 57 81 00  .T‹......t1. WЃ.
00 0a f9 62 00 00 01 00 00 00 00 00 00 00 00 00  ..щb............
00 00 86 8a 00 00 f7 4f 00 00 00 00 00 00 0c 00  ..†Љ..чO........
00 00 46 00 00 00 8a 00 00 00 86 00 00 00 3a ff  ..F...Љ...†...:я
bf f3 8f 01 00 00 1a 00 00 00 00 12 4b 00 0c 46  їуЏ.........K..F
8a 86 9c 8b 01 10 c0 8b 01 10 78 8b 01 10 0c 46  Љ†њ‹..А‹..x‹...F
8a 86 0c 00 00 00 03 00 00 00 04 00 00 00 e3 0c  Љ†............г.
00 00 03 00 00 00 03 00 00 00 00 02 00 00 3a ff  ..............:я
bf f3 25 00 00 00 a5 13 00 00 3a ff bf f3 e9 0d  їу%...Ґ...:яїуй.
00 00 0a 08 ed 00 e0 00 10 00 50 09 00 00 00 00  ....н.а...P.....
10 00 50 00 10 09 40 1f 0b 00 00 00 00 00 00 ff  ..P...@........я
ff ff ff 0c 46 8a 86 01 00 00 01 00 02 00 f0 1e  яяя.FЉ†.......р.
52 04 0c 0a 0a 10 0e 80 03 80 00 00 00 00 00 00  R......Ђ.Ђ......
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
00 00 00 fd 00 00 00 00 00 00 00 00 00 00 00 00  ...э............
00 00 00 00 00 00 00 40 40 00 00 00 00 00 00 00  .......@@.......
00 00 00 4a 39 00 20 9b 00 00 00 01 00 00 00 60  ...J9. ›.......`
3d 00 20 0a d0 39 00 20 77 31 00 20 78 31 00 20  =. .Р9. w1. x1. 
58 3d 00 20 00 00 00 00 17 01 01 00 28 00 00 00  X=. ........(...
c0 39 00 20 c4 39 00 20 93 96 00 00 74 00 00 00  А9. Д9. “–..t...
44 06 00 20 c0 39 00 20 0d 9c 00 00 d0 39 00 20  D.. А9. .њ..Р9. 
00 00 00 00 28 00 00 00 74 00 00 00 00 00 00 00  ....(...t.......
a1 9c 00 00 e0 0d 00 20 38 06 00 20 02 00 00 00  Ўњ..а.. 8.. ....
59 80 00 00 e0 0d 00 20 08 07 00 20 08 31 00 20  YЂ..а.. ... .1. 
51 82 00 00 00 00 00 00 f0 30 00 20 00 00 00 00  Q‚......р0. ....
31 9b 00 00 0a 00 00 00 00 60 09 00 20 60 09 00  1›.......`.. `..
20 34 05 00 20 34 05 00 20 bd 53 00 00 e0 0d 00   4.. 4.. ЅS..а..
20 00 09 00 20 00 09 00 20 54 31 00 20 54 31 00   ... ... T1. T1.
20 0f 31 00 00 cc 05 00 20 68 3f 00 20 68 3f 00   .1..М.. h?. h?.
20 db 7d 00 00 e0 0d 00 20 cc 05 00 20 82 00 00   Ы}..а.. М.. ‚..
00 a5 7d 00 00 0c 00 00 00 f0 18 01 00 00 00 09  .Ґ}......р......
40 59 80 00 00 03 05 00 20 00 10 00 20 0c 8b 01  @YЂ..... ... .‹.
10 30 8b 01 10 54 8b 01 10 01 00 00 00 74 31 00  .0‹..T‹......t1.
20 57 81 00 00 0a f9 62 00 00 01 00 00 00 00 00   WЃ...щb........
00 00 00 00 00 00 86 8a 00 00 f7 4f 00 00 00 00  ......†Љ..чO....
00 00 0c 00 00 00 46 00 00 00 8a 00 00 00 86 00  ......F...Љ...†.
00 00 3a ff bf f3 8f 01 00 00 1a 00 00 00 00 12  ..:яїуЏ.........
4b 00 0c 46 8a 86 9c 8b 01 10 c0 8b 01 10 78 8b  K..FЉ†њ‹..А‹..x‹
01 10 0c 46 8a 86 0c 00 00 00 03 00 00 00 04 00  ...FЉ†..........
00 00 e3 0c 00 00 03 00 00 00 03 00 00 00 00 02  ..г.............
00 00 3a ff bf f3 25 00 00 00 a5 13 00 00 3a ff  ..:яїу%...Ґ...:я
bf f3 e9 0d 00 00 0a                             їуй....         

0a 0a 2f 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d  ../-------------
2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d  ----------------
2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d  ----------------
2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2d 2f 0a  --------------/.
74 65 73 74 20 62 6f 6f 74 0a 70 72 69 76 61 74  test boot.privat
65 4b 65 79 2d 3e 
d0 39 00 20 77 31 00 20 78 31  eKey->Р9. w1. x1
00 20 58 3d 00 20 00 00 00 00 17 01 01 00 28 00  . X=. ........(.
00 00 c0 39 00 20 c4 39 00 20 93 96 00 00 74 00  ..А9. Д9. “–..t.
00 00 44 06 00 20 c0 39 00 20 0d 9c 00 00 d0 39  ..D.. А9. .њ..Р9
00 20 00 00 00 00 28 00 00 00 74 00 00 00 00 00  . ....(...t.....
00 00 a1 9c 00 00 e0 0d 00 20 38 06 00 20 02 00  ..Ўњ..а.. 8.. ..
00 00 59 80 00 00 e0 0d 00 20 08 07 00 20 08 31  ..YЂ..а.. ... .1
00 20 51 82 00 00 00 00 00 00 f0 30 00 20 00 00  . Q‚......р0. ..
00 00 31 9b 00 00 0a 70 75 62 6c 69 63 4b 65 79  ..1›...publicKey
5f 78 2d 3e 

00 00 00 00 60 09 00 20 60 09 00 20  _x->....`.. `.. 
34 05 00 20 34 05 00 20 bd 53 00 00 e0 0d 00 20  4.. 4.. ЅS..а.. 
00 09 00 20 00 09 00 20 54 31 00 20 54 31 00 20  ... ... T1. T1. 
0f 31 00 00 cc 05 00 20 68 3f 00 20 68 3f 00 20  .1..М.. h?. h?. 
db 7d 00 00 e0 0d 00 20 cc 05 00 20 82 00 00 00  Ы}..а.. М.. ‚...
a5 7d 00 00 0c 00 00 00 f0 18 01 00 00 00 09 40  Ґ}......р......@
59 80 00 00 03 05 00 20 00 10 00 20 0c 8b 01 10  YЂ..... ... .‹..
30 8b 01 10 54 8b 01 10 01 00 00 00 74 31 00 20  0‹..T‹......t1. 
57 81 00 00 

0a 70 75 62 6c 69 63 4b 65 79 5f 79  WЃ...publicKey_y
2d 3e 
f9 62 00 00 01 00 00 00 00 00 00 00 00 00  ->щb............
00 00 86 8a 00 00 f7 4f 00 00 00 00 00 00 0c 00  ..†Љ..чO........
00 00 46 00 00 00 8a 00 00 00 86 00 00 00 3a ff  ..F...Љ...†...:я
bf f3 8f 01 00 00 1a 00 00 00 00 12 4b 00 0c 46  їуЏ.........K..F
8a 86 9c 8b 01 10 c0 8b 01 10 78 8b 01 10 0c 46  Љ†њ‹..А‹..x‹...F
8a 86 0c 00 00 00 03 00 00 00 04 00 00 00 83 0d  Љ†............ѓ.
00 00 03 00 00 00 03 00 00 00 00 02 00 00 3a ff  ..............:я
bf f3 25 00 00 00 45 14 00 00 3a ff bf f3 89 0e  їу%...E...:яїу‰.
00 00 0a 68 61 73 68 2d 3e 
08 ed 00 e0 00 10 00  ...hash->.н.а...
50 09 00 00 00 00 10 00 50 00 10 09 40 bf 0b 00  P.......P...@ї..
00 00 00 00 00 ff ff ff ff 0c 46 8a 86 01 00 00  .....яяяя.FЉ†...
01 00 02 00 f0 1e 52 04 0c 0a 0a 10 0e 80 03 80  ....р.R......Ђ.Ђ
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
00 00 00 00 00 00 00 00 00 fd 00 00 00 00 00 00  .........э......
00 00 00 00 00 00 00 00 00 00 00 00 00 40 40 00  .............@@.
00 00 00 00 00 00 00 00 00 4a 39 00 20 9b 00 00  .........J9. ›..
00 01 00 00 00 60 3d 00 20 0a 6b 2d 3e 
d0 39 00  .....`=. .k->Р9.
20 77 31 00 20 78 31 00 20 58 3d 00 20 00 00 00   w1. x1. X=. ...
00 17 01 01 00 28 00 00 00 c0 39 00 20 c4 39 00  .....(...А9. Д9.
20 93 96 00 00 74 00 00 00 44 06 00 20 c0 39 00   “–..t...D.. А9.
20 0d 9c 00 00 d0 39 00 20 00 00 00 00 28 00 00   .њ..Р9. ....(..
00 74 00 00 00 00 00 00 00 a1 9c 00 00 e0 0d 00  .t.......Ўњ..а..
20 38 06 00 20 02 00 00 00 59 80 00 00 e0 0d 00   8.. ....YЂ..а..
20 08 07 00 20 08 31 00 20 51 82 00 00 00 00 00   ... .1. Q‚.....
00 f0 30 00 20 00 00 00 00 31 9b 00 00 0a 72 2d  .р0. ....1›...r-
3e 
00 00 00 00 60 09 00 20 60 09 00 20 34 05 00  >....`.. `.. 4..
20 34 05 00 20 bd 53 00 00 e0 0d 00 20 00 09 00   4.. ЅS..а.. ...
20 00 09 00 20 54 31 00 20 54 31 00 20 0f 31 00   ... T1. T1. .1.
00 cc 05 00 20 68 3f 00 20 68 3f 00 20 db 7d 00  .М.. h?. h?. Ы}.
00 e0 0d 00 20 cc 05 00 20 82 00 00 00 a5 7d 00  .а.. М.. ‚...Ґ}.
00 0c 00 00 00 f0 18 01 00 00 00 09 40 59 80 00  .....р......@YЂ.
00 03 05 00 20 00 10 00 20 0c 8b 01 10 30 8b 01  .... ... .‹..0‹.
10 54 8b 01 10 01 00 00 00 74 31 00 20 57 81 00  .T‹......t1. WЃ.
00 0a 73 2d 3e 
f9 62 00 00 01 00 00 00 00 00 00  ..s->щb.........
00 00 00 00 00 86 8a 00 00 f7 4f 00 00 00 00 00  .....†Љ..чO.....
00 0c 00 00 00 46 00 00 00 8a 00 00 00 86 00 00  .....F...Љ...†..
00 3a ff bf f3 8f 01 00 00 1a 00 00 00 00 12 4b  .:яїуЏ.........K
00 0c 46 8a 86 9c 8b 01 10 c0 8b 01 10 78 8b 01  ..FЉ†њ‹..А‹..x‹.
10 0c 46 8a 86 0c 00 00 00 03 00 00 00 04 00 00  ..FЉ†...........
00 83 0d 00 00 03 00 00 00 03 00 00 00 00 02 00  .ѓ..............
00 3a ff bf f3 25 00 00 00 45 14 00 00 3a ff bf  .:яїу%...E...:яї
f3 89 0e 00 00 0a                                у‰....          









	//x = 6F F5 1D 33 18 D0 D2 DC 6D 54 D4 42 3E 5E 13 34 F3 69 8B 3E 99 03 48 67 A4 77 19 F1 2F DE BE 6C
	license.l_public.x[0] = 0x2FDEBE6C;
	license.l_public.x[1] = 0xA47719F1;
	license.l_public.x[2] = 0x99034867;
	license.l_public.x[3] = 0xF3698B3E;
	license.l_public.x[4] = 0x3E5E1334;
	license.l_public.x[5] = 0x6D54D442;
	license.l_public.x[6] = 0x18D0D2DC;
	license.l_public.x[7] = 0x6FF51D33;
	
	//y = AE F2 B7 CD 47 37 41 83 9B 96 F6 A5 A4 D8 C0 8B CC 76 9E 3E 14 12 9C C5 06 FC C1 EB 0C 36 BF 5C
	license.l_public.y[0] = 0x0C36BF5C;
	license.l_public.y[1] = 0x06FCC1EB;
	license.l_public.y[2] = 0x14129CC5;
	license.l_public.y[3] = 0xCC769E3E;
	license.l_public.y[4] = 0xA4D8C08B;
	license.l_public.y[5] = 0x9B96F6A5;
	license.l_public.y[6] = 0x47374183;
	license.l_public.y[7] = 0xAEF2B7CD;

	//hash = 86 8A 46 0C 00 4B 12 00 86 8A 46 0C 00 4B 12 00 86 8A 46 0C 00 4B 12 00 86 8A 46 0C 00 4B 12 00
	license.l_hash[0] = 0x004B1200;
	license.l_hash[1] = 0x868A460C;
	license.l_hash[2] = 0x004B1200;
	license.l_hash[3] = 0x868A460C;
	license.l_hash[4] = 0x004B1200;
	license.l_hash[5] = 0x868A460C;
	license.l_hash[6] = 0x004B1200;
	license.l_hash[7] = 0x868A460C;
	
	//r = 87 AE C4 33 E5 AC 57 32 69 E3 48 09 F3 6E D7 2D 2A DC 71 5B 1A 79 6B 4B 47 0A AC 17 54 4D 75 96	
	license.r[0] = 0x544D7596;
	license.r[1] = 0x470AAC17;
	license.r[2] = 0x1A796B4B;
	license.r[3] = 0x2ADC715B;
	license.r[4] = 0xF36ED72D;
	license.r[5] = 0x69E34809;
	license.r[6] = 0xE5AC5732;
	license.r[7] = 0x87AEC433;
	
	//s = D9 8F D4 43 D2 CC 7C B9 AB 85 88 8A 9A 14 05 E0 EE 4D DF ED E4 85 69 26 EE 04 E8 5B 06 3A 7D 56
	license.s[0] = 0x063A7D56;
	license.s[1] = 0xEE04E85B;
	license.s[2] = 0xE4856926;
	license.s[3] = 0xEE4DDFED;
	license.s[4] = 0x9A1405E0;
	license.s[5] = 0xAB85888A;
	license.s[6] = 0xD2CC7CB9;
	license.s[7] = 0xD98FD443;



//x = 6F  F5 1D 33 18 D0 D2 DC 6D 54 D4 42 3E 5E 13 34 F3 69 8B 3E 99 03 48 67 A4 77 19 F1 2F DE BE 6C
license.l_public.x[0] = 0x2FDEBE6C;
license.l_public.x[1] = 0xA47719F1;
license.l_public.x[2] = 0x99034867;
license.l_public.x[3] = 0xF3698B3E;
license.l_public.x[4] = 0x3E5E1334;
license.l_public.x[5] = 0x6D54D442;
license.l_public.x[6] = 0x18D0D2DC;
license.l_public.x[7] = 0x6FF51D33;

6c be de 2f f1 19 77 a4 67 48 03 99 3e 8b 69 f3  lѕЮ/с.w¤gH.™>‹iу
34 13 5e 3e 42 d4 54 6d dc d2 d0 18 33 1d f5 6f  4.^>BФTmЬТР.3.хo

5c bf 36 0c eb c1 fc 06 c5 9c 12 14 3e 9e 76 cc  \ї6.лБь.Ењ..>ћvМ
8b c0 d8 a4 a5 f6 96 9b 83 41 37 47 cd b7 f2 ae  ‹АШ¤Ґц–›ѓA7GН·т®

00 12 4b 00 0c 46 8a 86 00 12 4b 00 0c 46 8a 86  ..K..FЉ†..K..FЉ†
00 12 4b 00 0c 46 8a 86 00 12 4b 00 0c 46 8a 86  ..K..FЉ†..K..FЉ†

96 75 4d 54 17 ac 0a 47 4b 6b 79 1a 5b 71 dc 2a  –uMT.¬.GKky.[qЬ*
2d d7 6e f3 09 48 e3 69 32 57 ac e5 33 c4 ae 87  -Чnу.Hгi2W¬е3Д®‡

56 7d 3a 06 5b e8 04 ee 26 69 85 e4 ed df 4d ee  V}:.[и.о&i…днЯMо
e0 05 14 9a 8a 88 85 ab b9 7c cc d2 43 d4 8f d9  а..љЉ€…«№|МТCФЏЩ




















:20 0000 00 0050002075110000D91F0000DB1F0000DD1F0000DD1F0000DD1F000000000000 04






6c be de 2f f1 19 77 a4 67 48 03 99 3e 8b 69 f3  
34 13 5e 3e 42 d4 54 6d dc d2 d0 18 33 1d f5 6f  
5c bf 36 0c eb c1 fc 06 c5 9c 12 14 3e 9e 76 cc 
8b c0 d8 a4 a5 f6 96 9b 83 41 37 47 cd b7 f2 ae  
00 12 4b 00 0c 46 8a 86 00 12 4b 00 0c 46 8a 86  
00 12 4b 00 0c 46 8a 86 00 12 4b 00 0c 46 8a 86 
96 75 4d 54 17 ac 0a 47 4b 6b 79 1a 5b 71 dc 2a 
2d d7 6e f3 09 48 e3 69 32 57 ac e5 33 c4 ae 87 
56 7d 3a 06 5b e8 04 ee 26 69 85 e4 ed df 4d ee 
e0 05 14 9a 8a 88 85 ab b9 7c cc d2 43 d4 8f d9 





:020000040001F9
:20D000006CBEDE2FF11977A4674803993E8B69F334135E3E42D4546DDCD2D018331DF56F40
:20D020005CBF360CEBC1FC06C59C12143E9E76CC8BC0D8A4A5F6969B83413747CDB7F2AE47
:20D0400000124B000C468A8600124B000C468A8600124B000C468A8600124B000C468A86D4
:20D0600096754D5417AC0A474B6B791A5B71DC2A2DD76EF30948E3693257ACE533C4AE878D
:20D08000567D3A065BE804EE266985E4EDDF4DEEE005149A8A8885ABB97CCCD243D48FD922 
:00000001FF


:20 D08000567D3A065BE804EE266985E4EDDF4DEEE005149A8A8885ABB97CCCD243D48FD922 

20 D0 80 00 56 7d 3a 06 5b e8 04 ee 26 69 85 e4 ed df 4d ee e0 05 14 9a 8a 88 85 ab b9 7c cc d2 43 d4 8f d9




	l_hash[0] = 0x004B1200;
	l_hash[1] = 0x868A460C;
	l_hash[2] = 0x004B1200;
	l_hash[3] = 0x868A460C;
	l_hash[4] = 0x004B1200;
	l_hash[5] = 0x868A460C;
	l_hash[6] = 0x004B1200;
	l_hash[7] = 0x868A460C;
	
	
	
	
	license_t license;
	read_license((uint8_t*)(&license), 128);
	
	hexraw_print(8, ((uint8_t*)(0x500012F0)));
	hexraw_print(32, (uint8_t*)(&l_hash));
	
	l_hash[0] = ((uint32_t*)(0x500012F0));
	l_hash[1] = ((uint32_t*)(0x500012F4));
	l_hash[2] = ((uint32_t*)(0x500012F0));
	l_hash[3] = ((uint32_t*)(0x500012F4));
	l_hash[4] = ((uint32_t*)(0x500012F0));
	l_hash[5] = ((uint32_t*)(0x500012F4));
	l_hash[6] = ((uint32_t*)(0x500012F0));
	l_hash[7] = ((uint32_t*)(0x500012F4));
	hexraw_print(32, (uint8_t*)(&l_hash));






00 12 4b 00 0c 46 8a 86 00 12 4b 00 0c 46 8a 86 00 12 4b 00 0c 46 8a 86 00 12 4b 00 0c 46 8a 86 




86 8a 46 0c 00 4b 12 00
00 4b 12 00 86 8a 46 0c
 


00 4b 12 00 86 8a 46 0c 
00 4b 12 00 86 8a 46 0c 
00 4b 12 00 86 8a 46 0c 
00 4b 12 00 86 8a 46 0c 


/*COMP*/
//x = 6F F5 1D 33 18 D0 D2 DC 6D 54 D4 42 3E 5E 13 34 F3 69 8B 3E 99 03 48 67 A4 77 19 F1 2F DE BE 6C
6c be de 2f f1 19 77 a4 67 48 03 99 3e 8b 69 f3 
34 13 5e 3e 42 d4 54 6d dc d2 d0 18 33 1d f5 6f  

//y = AE F2 B7 CD 47 37 41 83 9B 96 F6 A5 A4 D8 C0 8B CC 76 9E 3E 14 12 9C C5 06 FC C1 EB 0C 36 BF 5C
5c bf 36 0c eb c1 fc 06 c5 9c 12 14 3e 9e 76 cc 
8b c0 d8 a4 a5 f6 96 9b 83 41 37 47 cd b7 f2 ae 

//r = 87 AE C4 33 E5 AC 57 32 69 E3 48 09 F3 6E D7 2D 2A DC 71 5B 1A 79 6B 4B 47 0A AC 17 54 4D 75 96
96 75 4d 54 17 ac 0a 47 4b 6b 79 1a 5b 71 dc 2a  
2d d7 6e f3 09 48 e3 69 32 57 ac e5 33 c4 ae 87  

//s = B7 41 11 8E B5 DD 6D B3 37 89 E9 17 A0 1B 2A D6 F6 8C 96 44 82 42 05 F2 B3 C9 A8 30 31 42 DB 00
00 db 42 31 30 a8 c9 b3 f2 05 42 82 44 96 8c f6 
d6 2a 1b a0 17 e9 89 37 b3 6d dd b5 8e 11 41 b7 



00 4b 12 00 86 8a 46 0c //befor  
86 8a 46 0c 00 4b 12 00 //after 




6c be de 2f f1 19 77 a4 67 48 03 99 3e 8b 69 f3  lѕЮ/с.w¤gH.™>‹iу
34 13 5e 3e 42 d4 54 6d dc d2 d0 18 33 1d f5 6f  4.^>BФTmЬТР.3.хo
5c bf 36 0c eb c1 fc 06 c5 9c 12 14 3e 9e 76 cc  \ї6.лБь.Ењ..>ћvМ
8b c0 d8 a4 a5 f6 96 9b 83 41 37 47 cd b7 f2 ae  ‹АШ¤Ґц–›ѓA7GН·т®
96 75 4d 54 17 ac 0a 47 4b 6b 79 1a 5b 71 dc 2a  –uMT.¬.GKky.[qЬ*
2d d7 6e f3 09 48 e3 69 32 57 ac e5 33 c4 ae 87  -Чnу.Hгi2W¬е3Д®‡
00 db 42 31 30 a8 c9 b3 f2 05 42 82 44 96 8c f6  .ЫB10ЁЙіт.B‚D–Њц
d6 2a 1b a0 17 e9 89 37 b3 6d dd b5 8e 11 41 b7  Ц*. .й‰7іmЭµЋ.A·




  HWREG(NVIC_VTABLE) = (0x3000 + 0x100);//OTA_IMAGE_OFFSET + OTA_METADATA_SPACE;



  
  
  

0x00000000 //bootloader
0x00003000 //metadata
0x00003100 //program
0x0001D000 //licence
0x0001E000 //eeprom

0x55 0x55 //start
0x03 0x20 0x20 //COMMAND_PING
0x03 0x23 0x23 //COMMAND_GET_STATUS
0x03 0x28 0x28 //COMMAND_GET_CHIP_ID
0x09 0x82 0x2a 0x50 0x00 0x12 0xf4 0x01 0x01 //COMMAND_MEMORY_READ ADDR: MAC_15_4_1
0x09 0x7e 0x2a 0x50 0x00 0x12 0xf0 0x01 0x01 //COMMAND_MEMORY_READ ADDR: MAC_15_4_0						


0x09 0x2C 0x2A 0x00 0x00 0x00 0x00 0x01 0x01 //READ ADDR:0x00000000
//0x0D 0x2D 0x2B 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x01 0x01 //WRITE ADDR:0x00000000 = 0x00000000


0x09 0x32 0x2A 0x00 0x00 0x00 0x00 0x04 0x04
0x09 0x2F 0x2A 0x00 0x00 0x00 0x00 0x04 0x01



unsigned char ucCommand[6];
ucCommand[0]= COMMAND_MEMORY_READ;
ucCommand[1]= Data Address [31:24];
ucCommand[2]= Data Address [23:16];
ucCommand[3]= Data Address [15: 8];
ucCommand[4]= Data Address [ 7: 0];
ucCommand[5]= Read access width [7:0];

0x08 0x2B 0x2A 0x00 0x00 0x00 0x00 0x01 //COMMAND_MEMORY_READ

55 55 						//Auto-Baud
03 28 28 					//COMMAND_GET_CHIP_ID
00 cc						//ACK
03 23 23 					//COMMAND_GET_STATUS
00 cc 						//ACK
09 a7 2a 50 00 13 18 01 01	//COMMAND_MEMORY_READ
00 cc 						//ACK
03 23 23 					//COMMAND_GET_STATUS
00 cc 						//ACK
09 22 2a 					//COMMAND_MEMORY_READ 
50 00 12 94						//ADDR: USER_ID
01 01 							//
00 cc 						//ACK
03 23 23 					//COMMAND_GET_STATUS
00 cc 						//ACK
09 2c 2a 					//COMMAND_MEMORY_READ
50 00 10 a0 					//ADDR: MISC_CONF_1
01 01 							//
00 cc 						//ACK
03 23 23 					//COMMAND_GET_STATUS
00 cc 						//ACK
09 9b 2a 					//COMMAND_MEMORY_READ
40 03 00 2c 					//ADDR: (FLASH Flash Controller 0x4003 0000)
01 01 							//
00 cc 						//ACK
03 23 23 					//COMMAND_GET_STATUS
00 cc 						//ACK
09 e6 2a 					//COMMAND_MEMORY_READ
40 08 22 50 					//ADDR: (PRCM Power, Clock, and Reset Management 0x4008 2000)
01 01 							//
00 cc 						//ACK
03 23 23 					//COMMAND_GET_STATUS
00 cc 						//ACK
09 82 2a 					//COMMAND_MEMORY_READ
50 00 12 f4 					//ADDR: MAC_15_4_1
01 01 							//
00 cc 						//ACK
03 23 23 					//COMMAND_GET_STATUS
00 cc 						//ACK
09 7e 2a 					//COMMAND_MEMORY_READ
50 00 12 f0 					//ADDR: MAC_15_4_0
01 01 							//
00 cc 						//ACK
03 23 23 					//COMMAND_GET_STATUS
00 cc 						//ACK
03 2c 2c 					//??????????????????
03 23 23 					//COMMAND_GET_STATUS
00 cc 						//ACK
0b a1 21 					//COMMAND_DOWNLOAD
00 00 00 00 00 00 00 80 		//DATA
03 23 23 					//COMMAND_GET_STATUS
00 cc 						//ACK

83 3d						//COMMAND_SEND_DATA
24 6c be de 2f f1 19 77 a4 67 48 03 99 3e 8b 69 
f3 34 13 5e 3e 42 d4 54 6d dc d2 d0 18 33 1d f5 
6f 5c bf 36 0c eb c1 fc 06 c5 9c 12 14 3e 9e 76 
cc 8b c0 d8 a4 a5 f6 96 9b 83 41 37 47 cd b7 f2 
ae 96 75 4d 54 17 ac 0a 47 4b 6b 79 1a 5b 71 dc 
2a 2d d7 6e f3 09 48 e3 69 32 57 ac e5 33 c4 ae 
87 00 db 42 31 30 a8 c9 b3 f2 05 42 82 44 96 8c 
f6 d6 2a 1b a0 17 e9 89 37 b3 6d dd b5 8e 11 41 
b7 

03 23 23 					//COMMAND_GET_STATUS
00 cc 						//ACK
0f a7 27 00 00 00 00 00 00 00 80 00 00 00 00 //COMMAND_CRC32
00 cc 						//ACK
03 23 23 					//COMMAND_GET_STATUS
00 cc 						//ACK
03 25 25                    //COMMAND_RESET       























55 55 03 28 28 00 cc 03 23 23 00 cc 09 a7 2a 50  UU.((.М.##.М.§*P
00 13 18 01 01 00 cc 03 23 23 00 cc 09 22 2a 50  ......М.##.М."*P
00 12 94 01 01 00 cc 03 23 23 00 cc 09 2c 2a 50  ..”...М.##.М.,*P
00 10 a0 01 01 00 cc 03 23 23 00 cc 09 9b 2a 40  .. ...М.##.М.›*@
03 00 2c 01 01 00 cc 03 23 23 00 cc 09 e6 2a 40  ..,...М.##.М.ж*@
08 22 50 01 01 00 cc 03 23 23 00 cc 09 82 2a 50  ."P...М.##.М.‚*P
00 12 f4 01 01 00 cc 03 23 23 00 cc 09 7e 2a 50  ..ф...М.##.М.~*P
00 12 f0 01 01 00 cc 03 23 23 00 cc 03 2c 2c 03  ..р...М.##.М.,,.
23 23 00 cc 0b a1 21 00 00 00 00 00 00 00 80 03  ##.М.Ў!.......Ђ.
23 23 00 cc 83 3d 24 6c be de 2f f1 19 77 a4 67  ##.Мѓ=$lѕЮ/с.w¤g
48 03 99 3e 8b 69 f3 34 13 5e 3e 42 d4 54 6d dc  H.™>‹iу4.^>BФTmЬ
d2 d0 18 33 1d f5 6f 5c bf 36 0c eb c1 fc 06 c5  ТР.3.хo\ї6.лБь.Е
9c 12 14 3e 9e 76 cc 8b c0 d8 a4 a5 f6 96 9b 83  њ..>ћvМ‹АШ¤Ґц–›ѓ
41 37 47 cd b7 f2 ae 96 75 4d 54 17 ac 0a 47 4b  A7GН·т®–uMT.¬.GK
6b 79 1a 5b 71 dc 2a 2d d7 6e f3 09 48 e3 69 32  ky.[qЬ*-Чnу.Hгi2
57 ac e5 33 c4 ae 87 00 db 42 31 30 a8 c9 b3 f2  W¬е3Д®‡.ЫB10ЁЙіт
05 42 82 44 96 8c f6 d6 2a 1b a0 17 e9 89 37 b3  .B‚D–ЊцЦ*. .й‰7і
6d dd b5 8e 11 41 b7 03 23 23 00 cc 0f a7 27 00  mЭµЋ.A·.##.М.§'.
00 00 00 00 00 00 80 00 00 00 00 00 cc 03 23 23  ......Ђ.....М.##
00 cc 03 25 25                                   .М.%%           





03 23 23 										//COMMAND_GET_STATUS
00 cc 											//ACK
0b 41 21 00 00 00 00 00 00 00 20 				//COMMAND_DOWNLOAD
03 23 23 										//COMMAND_GET_STATUS
00 cc											//ACK

23 f4 24 6c be de 2f f1 19 77 a4 67 48 03 99 3e //COMMAND_SEND_DATA
8b 69 f3 34 13 5e 3e 42 d4 54 6d dc d2 d0 18 33 
1d f5 6f 

03 23 23 										//COMMAND_GET_STATUS
00 cc 											//ACK
0f 47 27 00 00 00 00 00 00 00 20 00 00 00 00 	//COMMAND_CRC32
00 cc 											//ACK
03 23 23 										//COMMAND_GET_STATUS
00 cc 											//ACK
03 25 25     									//COMMAND_GET_STATUS                  
                                    






55 55 
03 28 28 
00 cc 
03 23 23 
00 cc 
09 a7 2a 50 00 13 18 01 01 
00 cc 
03 23 23 
00 cc 
09 22 2a 50 00 12 94 01 01 
00 cc 
03 23 23 
00 cc 
09 2c 2a 50 00 10 a0 01 01 
00 cc 
03 23 23 
00 cc 
09 9b 2a 40 03 00 2c 01 01 
00 cc 
03 23 23 
00 cc 
09 e6 2a 40 08 22 50 01 01 
00 cc 
03 23 23 
00 cc 
09 82 2a 50 00 12 f4 01 01 
00 cc 
03 23 23 
00 cc 
09 7e 2a 50 00 12 f0 01 01 
00 cc 
03 23 23 
00 cc 
0b 41 21 00 00 00 00 00 00 00 20 //COMMAND_DOWNLOAD
03 23 23
00 cc 

23 f4 24 
6c be de 2f f1 19 77 a4 67 48 03 99 3e 8b 69 f3 
34 13 5e 3e 42 d4 54 6d dc d2 d0 18 33 1d f5 6f 

03 23 23 
00 cc                                     


//Auto-Baud
0x55 0x55
//COMMAND_DOWNLOAD
0x0b 0x41 0x21 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x20
//COMMAND_GET_STATUS  
0x03 0x23 0x23
//ACK
0x00 0xcc 
//COMMAND_SEND_DATA
0x23 0xf4 0x24 0x6c 0xbe 0xde 0x2f 0xf1 0x19 0x77 0xa4 0x67 0x48 0x03 0x99 0x3e 0x8b 0x69 0xf3 0x34 0x13 0x5e 0x3e 0x42 0xd4 0x54 0x6d 0xdc 0xd2 0xd0 0x18 0x33 0x1d 0xf5 0x6f 
//COMMAND_GET_STATUS  
0x03 0x23 0x23



0x09 0x2C 0x2A 0x00 0x00 0x00 0x00 0x01 0x01 //READ ADDR:0x00000000
//0x0D 0x2D 0x2B 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x01 0x01 //WRITE ADDR:0x00000000 = 0x00000000


0x1D000

0x09 0xFD 0x2A 0x00 0x01 0xD0 0x00 0x01 0x01 //READ 0x1D000
0x09 0xFE 0x2A 0x00 0x01 0xD0 0x01 0x01 0x01 //READ 0x1D001





0x09 0x82 0x2a 0x50 0x00 0x12 0xf4 0x01 0x01 //COMMAND_MEMORY_READ ADDR: MAC_15_4_1
0x09 0x7e 0x2a 0x50 0x00 0x12 0xf0 0x01 0x01 //COMMAND_MEMORY_READ ADDR: MAC_15_4_0		

0x09 0xFE 0x2a 0x00 0x01 0xD0 0x01 0x01 0x01 
0x0D 0x2D 0x2B 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x01 0x01


srec_cat.exe 
simple_peripheral_observer_cc2650lp_stack.hex
-Intel 
simple_peripheral_observer_cc2650lp_app.hex 
-Intel 
-o 
cc2650lp_ble.hex 
-Intel

srec_cat 
firmware-metadata.bin 
-binary $$@.bin 
-binary 
-offset 
0x100 
-o 
$$@-ota-image.bin 
-binary


srec_cat 
../bootloader/bootloader.hex 
-intel 
-crop 
0x0 
0x3000 
0x1FFA8 
0x20000 
$$@-ota-image.bin 
-binary 
-offset 
0x3000 
-crop 
0x3000 
0x1B000 
-o 
$$@-firmware.hex -intel
	


srec_cat.exe root-firmware.hex -intel license.bin -binary -offset 0x1D000 -o firmware.hex -intel







>0x55 0x55 
<0x00 0xcc
>0x09 0x7e 0x2a 0x50 0x00 0x12 0xf0 0x01 0x01 //COMMAND_MEMORY_READ ADDR: MAC_15_4_0	
<0x00 0xcc 0x06 0x62 0x86 0x8a 0x46 0x0c 
>0x00 0xcc
>0x09 0x82 0x2a 0x50 0x00 0x12 0xf4 0x01 0x01 //COMMAND_MEMORY_READ ADDR: MAC_15_4_1
<0x00 0xcc 0x06 0x5d 0x00 0x4b 0x12 0x00
>0x00 0xcc




00:12:4b:00:0c:46:8a:86






typedef  struct {
	uint16_t panid; 				//+
    uint8_t channel; 				//+
	uint8_t interface; 				//+
	uint8_t aes_key[16];			//+
	uint32_t serial;				//+-
	uint8_t interface_configured;	//+
	uint8_t aes_key_configured;		//+
	uint8_t serial_configured;		//+-
}eeprom_t;

BBAA1A0111223344556677889900AABBCCDDEEFFF6920000000000FF

BBAA
1A
01
11223344556677889900AABBCCDDEEFF
F6920000
00
00
00
FF















































