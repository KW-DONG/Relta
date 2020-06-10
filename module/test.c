#include "test.h"
#include "config.h"
#include "type.h"
#include "buffer.h"

//preloaded path

const float path_0[7][4] = {{0.f,0.f,0.f,20.f}
                           ,{90.f,90.f,0.f,20.f}
                           ,{-90.f,90.f,0.f,20.f}
                           ,{-90.f,-90.f,0.f,20.f}
                           ,{90.f,-90.f,0.f,20.f}
                           ,{0.f,0.f,0.f,20.f}
                           ,{0.f,0.f,30.f,20.f}};

const float path_1[7][4] = {{0.f,0.f,10.f,10.f}
                           ,{0.f,0.f,0.f,10.f}
                           ,{0.f,0.f,10.f,10.f}
                           ,{0.f,0.f,0.f,10.f}
                           ,{0.f,0.f,10.f,10.f}
                           ,{0.f,0.f,0.f,10.f}
                           ,{0.f,0.f,10.f,10.f}};

const float path_2[7][4] = {{90.f,0.f,10.f,10.f}
                           ,{90.f,0.f,0.f,10.f}
                           ,{-90.f,0.f,0.f,10.f}
                           ,{90.f,0.f,0.f,10.f}
                           ,{-90.f,0.f,0.f,10.f}
                           ,{90.f,0.f,0.f,10.f}
                           ,{0.f,0.f,0.f,10.f}};

const float path_3[7][4] = {{0.f,90.f,0.f,10.f}
                           ,{0.f,-90.f,0.f,10.f}
                           ,{0.f,90.f,0.f,10.f}
                           ,{0.f,-90.f,0.f,10.f}
                           ,{0.f,90.f,0.f,10.f}
                           ,{0.f,-90.f,0.f,10.f}
                           ,{0.f,90.f,0.f,10.f}};

//frequency, direction, step
const float block_path[564][3][3] = {{{200, 200, 200},{0, 0, 0},{1560, 1560, 1560}},
{{60, 150, 150},{0, 0, 0},{12, 27, 27}},
{{61, 151, 151},{0, 0, 0},{12, 27, 27}},
{{62, 152, 152},{0, 0, 0},{13, 27, 27}},
{{63, 153, 153},{0, 0, 0},{13, 28, 28}},
{{64, 154, 154},{0, 0, 0},{13, 28, 28}},
{{65, 155, 155},{0, 0, 0},{13, 28, 28}},
{{66, 156, 156},{0, 0, 0},{13, 28, 28}},
{{67, 157, 157},{0, 0, 0},{13, 28, 28}},
{{68, 158, 158},{0, 0, 0},{14, 28, 28}},
{{69, 159, 160},{0, 0, 0},{14, 29, 29}},
{{70, 160, 161},{0, 0, 0},{14, 29, 29}},
{{71, 162, 162},{0, 0, 0},{14, 29, 29}},
{{72, 163, 163},{0, 0, 0},{14, 29, 29}},
{{73, 164, 164},{0, 0, 0},{14, 29, 29}},
{{74, 165, 165},{0, 0, 0},{15, 30, 30}},
{{75, 166, 166},{0, 0, 0},{15, 30, 30}},
{{76, 167, 167},{0, 0, 0},{15, 30, 30}},
{{77, 168, 169},{0, 0, 0},{15, 30, 30}},
{{78, 169, 170},{0, 0, 0},{15, 30, 30}},
{{79, 171, 171},{0, 0, 0},{15, 30, 30}},
{{80, 172, 172},{0, 0, 0},{15, 31, 31}},
{{81, 173, 173},{0, 0, 0},{16, 31, 31}},
{{82, 174, 174},{0, 0, 0},{16, 31, 31}},
{{83, 175, 176},{0, 0, 0},{16, 31, 31}},
{{84, 177, 177},{0, 0, 0},{16, 31, 31}},
{{85, 178, 178},{0, 0, 0},{16, 32, 32}},
{{86, 179, 179},{0, 0, 0},{16, 32, 32}},
{{87, 180, 180},{0, 0, 0},{17, 32, 32}},
{{88, 181, 182},{0, 0, 0},{17, 32, 32}},
{{89, 183, 183},{0, 0, 0},{17, 32, 32}},
{{90, 184, 184},{0, 0, 0},{17, 33, 33}},
{{91, 185, 186},{0, 0, 0},{17, 33, 33}},
{{92, 187, 187},{0, 0, 0},{17, 33, 33}},
{{93, 188, 188},{0, 0, 0},{18, 33, 33}},
{{94, 189, 189},{0, 0, 0},{18, 34, 34}},
{{95, 191, 191},{0, 0, 0},{18, 34, 34}},
{{96, 192, 192},{0, 0, 0},{18, 34, 34}},
{{97, 193, 193},{0, 0, 0},{18, 34, 34}},
{{97, 195, 195},{0, 0, 0},{18, 34, 34}},
{{98, 196, 196},{0, 0, 0},{18, 35, 35}},
{{99, 197, 198},{0, 0, 0},{19, 35, 35}},
{{100, 199, 199},{0, 0, 0},{19, 35, 35}},
{{101, 200, 200},{0, 0, 0},{19, 35, 35}},
{{102, 202, 202},{0, 0, 0},{19, 36, 36}},
{{103, 203, 203},{0, 0, 0},{19, 36, 36}},
{{104, 204, 205},{0, 0, 0},{19, 36, 36}},
{{105, 206, 206},{0, 0, 0},{20, 36, 36}},
{{106, 207, 208},{0, 0, 0},{20, 37, 37}},
{{107, 209, 209},{0, 0, 0},{20, 37, 37}},
{{108, 211, 211},{0, 0, 0},{20, 37, 37}},
{{109, 212, 212},{0, 0, 0},{20, 37, 37}},
{{110, 214, 214},{0, 0, 0},{20, 38, 38}},
{{111, 215, 216},{0, 0, 0},{21, 38, 38}},
{{112, 217, 217},{0, 0, 0},{21, 38, 38}},
{{113, 218, 219},{0, 0, 0},{21, 38, 38}},
{{114, 220, 220},{0, 0, 0},{21, 39, 39}},
{{114, 222, 222},{0, 0, 0},{21, 39, 39}},
{{115, 224, 224},{0, 0, 0},{21, 39, 39}},
{{116, 225, 226},{0, 0, 0},{21, 39, 40}},
{{117, 227, 227},{0, 0, 0},{22, 40, 40}},
{{118, 229, 229},{0, 0, 0},{22, 40, 40}},
{{119, 231, 231},{0, 0, 0},{22, 40, 40}},
{{120, 232, 233},{0, 0, 0},{22, 41, 41}},
{{121, 234, 235},{0, 0, 0},{22, 41, 41}},
{{122, 236, 237},{0, 0, 0},{22, 41, 41}},
{{123, 238, 238},{0, 0, 0},{23, 42, 42}},
{{124, 240, 240},{0, 0, 0},{23, 42, 42}},
{{125, 242, 242},{0, 0, 0},{23, 42, 42}},
{{126, 244, 244},{0, 0, 0},{23, 43, 43}},
{{127, 246, 246},{0, 0, 0},{23, 43, 43}},
{{128, 248, 249},{0, 0, 0},{23, 43, 43}},
{{129, 250, 251},{0, 0, 0},{24, 44, 44}},
{{130, 253, 253},{0, 0, 0},{24, 44, 44}},
{{131, 255, 255},{0, 0, 0},{24, 44, 44}},
{{132, 257, 257},{0, 0, 0},{24, 45, 45}},
{{133, 259, 260},{0, 0, 0},{24, 45, 45}},
{{134, 262, 262},{0, 0, 0},{24, 45, 46}},
{{135, 264, 265},{0, 0, 0},{25, 46, 46}},
{{136, 267, 267},{0, 0, 0},{5, 26, 26}},
{{137, 269, 270},{0, 0, 0},{5, 27, 27}},
{{138, 272, 272},{0, 0, 0},{5, 27, 27}},
{{139, 274, 275},{0, 0, 0},{5, 28, 28}},
{{140, 277, 277},{0, 0, 0},{5, 28, 28}},
{{141, 280, 280},{0, 0, 0},{6, 28, 28}},
{{142, 283, 283},{0, 0, 0},{6, 29, 29}},
{{143, 285, 286},{0, 0, 0},{6, 29, 29}},
{{144, 288, 289},{0, 0, 0},{6, 30, 30}},
{{145, 292, 292},{0, 0, 0},{6, 30, 30}},
{{146, 295, 295},{0, 0, 0},{6, 31, 31}},
{{147, 298, 298},{0, 0, 0},{7, 31, 31}},
{{2, 212, 105},{0, 1, 1},{0, 21, 10}},
{{3, 210, 103},{0, 1, 1},{0, 21, 10}},
{{3, 207, 102},{0, 1, 1},{0, 20, 10}},
{{4, 205, 100},{0, 1, 1},{0, 20, 10}},
{{5, 203, 99},{0, 1, 1},{0, 20, 10}},
{{6, 200, 98},{0, 1, 1},{0, 20, 9}},
{{7, 198, 96},{0, 1, 1},{0, 19, 9}},
{{7, 196, 95},{0, 1, 1},{0, 19, 9}},
{{8, 194, 93},{0, 1, 1},{0, 19, 9}},
{{9, 191, 92},{0, 1, 1},{0, 19, 9}},
{{10, 189, 90},{0, 1, 1},{0, 19, 9}},
{{11, 187, 89},{0, 1, 1},{1, 18, 9}},
{{11, 185, 88},{0, 1, 1},{1, 18, 8}},
{{12, 183, 86},{0, 1, 1},{1, 18, 8}},
{{13, 181, 85},{0, 1, 1},{1, 18, 8}},
{{14, 179, 84},{0, 1, 1},{1, 18, 8}},
{{15, 177, 82},{0, 1, 1},{1, 17, 8}},
{{16, 175, 81},{0, 1, 1},{1, 17, 8}},
{{16, 173, 80},{0, 1, 1},{1, 17, 8}},
{{17, 171, 78},{0, 1, 1},{1, 17, 7}},
{{18, 170, 77},{0, 1, 1},{1, 17, 7}},
{{19, 168, 76},{0, 1, 1},{1, 16, 7}},
{{20, 166, 75},{0, 1, 1},{1, 16, 7}},
{{20, 164, 73},{0, 1, 1},{2, 16, 7}},
{{21, 163, 72},{0, 1, 1},{2, 16, 7}},
{{22, 161, 71},{0, 1, 1},{2, 16, 7}},
{{23, 159, 69},{0, 1, 1},{2, 16, 7}},
{{24, 157, 68},{0, 1, 1},{2, 15, 6}},
{{25, 156, 67},{0, 1, 1},{2, 15, 6}},
{{25, 154, 66},{0, 1, 1},{2, 15, 6}},
{{26, 153, 65},{0, 1, 1},{2, 15, 6}},
{{27, 151, 63},{0, 1, 1},{2, 15, 6}},
{{28, 149, 62},{0, 1, 1},{2, 15, 6}},
{{29, 148, 61},{0, 1, 1},{2, 14, 6}},
{{30, 146, 60},{0, 1, 1},{2, 14, 6}},
{{30, 145, 59},{0, 1, 1},{3, 14, 5}},
{{31, 143, 57},{0, 1, 1},{3, 14, 5}},
{{32, 142, 56},{0, 1, 1},{3, 14, 5}},
{{33, 140, 55},{0, 1, 1},{3, 14, 5}},
{{34, 139, 54},{0, 1, 1},{3, 13, 5}},
{{35, 137, 53},{0, 1, 1},{3, 13, 5}},
{{35, 136, 52},{0, 1, 1},{3, 13, 5}},
{{36, 134, 50},{0, 1, 1},{3, 13, 5}},
{{37, 133, 49},{0, 1, 1},{3, 13, 5}},
{{38, 132, 48},{0, 1, 1},{3, 13, 4}},
{{39, 130, 47},{0, 1, 1},{3, 13, 4}},
{{40, 129, 46},{0, 1, 1},{3, 13, 4}},
{{41, 128, 45},{0, 1, 1},{4, 12, 4}},
{{41, 126, 44},{0, 1, 1},{4, 12, 4}},
{{42, 125, 42},{0, 1, 1},{4, 12, 4}},
{{43, 124, 41},{0, 1, 1},{4, 12, 4}},
{{44, 122, 40},{0, 1, 1},{4, 12, 4}},
{{45, 121, 39},{0, 1, 1},{4, 12, 4}},
{{46, 120, 38},{0, 1, 1},{4, 12, 3}},
{{47, 118, 37},{0, 1, 1},{4, 11, 3}},
{{47, 117, 36},{0, 1, 1},{4, 11, 3}},
{{48, 116, 35},{0, 1, 1},{4, 11, 3}},
{{49, 115, 34},{0, 1, 1},{4, 11, 3}},
{{50, 113, 33},{0, 1, 1},{5, 11, 3}},
{{51, 112, 31},{0, 1, 1},{5, 11, 3}},
{{52, 111, 30},{0, 1, 1},{5, 11, 3}},
{{53, 110, 29},{0, 1, 1},{5, 11, 3}},
{{54, 109, 28},{0, 1, 1},{5, 10, 2}},
{{55, 107, 27},{0, 1, 1},{5, 10, 2}},
{{55, 106, 26},{0, 1, 1},{5, 10, 2}},
{{56, 105, 25},{0, 1, 1},{5, 10, 2}},
{{57, 104, 24},{0, 1, 1},{5, 10, 2}},
{{58, 103, 23},{0, 1, 1},{5, 10, 2}},
{{59, 102, 22},{0, 1, 1},{5, 10, 2}},
{{60, 100, 21},{0, 1, 1},{6, 10, 2}},
{{61, 99, 20},{0, 1, 1},{6, 10, 2}},
{{62, 98, 19},{0, 1, 1},{6, 9, 1}},
{{63, 97, 18},{0, 1, 1},{6, 9, 1}},
{{64, 96, 17},{0, 1, 1},{6, 9, 1}},
{{65, 95, 15},{0, 1, 1},{6, 9, 1}},
{{66, 94, 14},{0, 1, 1},{6, 9, 1}},
{{67, 93, 13},{0, 1, 1},{6, 9, 1}},
{{67, 92, 12},{0, 1, 1},{6, 9, 1}},
{{68, 91, 11},{0, 1, 1},{6, 9, 1}},
{{69, 89, 10},{0, 1, 1},{6, 9, 1}},
{{70, 88, 9},{0, 1, 1},{7, 8, 1}},
{{71, 87, 8},{0, 1, 1},{7, 8, 0}},
{{72, 86, 7},{0, 1, 1},{7, 8, 0}},
{{73, 85, 6},{0, 1, 1},{7, 8, 0}},
{{74, 84, 5},{0, 1, 1},{7, 8, 0}},
{{75, 83, 4},{0, 1, 1},{7, 8, 0}},
{{76, 82, 3},{0, 1, 1},{7, 8, 0}},
{{77, 81, 2},{0, 1, 1},{7, 8, 0}},
{{78, 80, 1},{0, 1, 1},{7, 8, 0}},
{{79, 79, 0},{0, 1, 1},{7, 8, 0}},
{{80, 78, 0},{0, 1, 0},{8, 7, 0}},
{{81, 77, 1},{0, 1, 0},{8, 7, 0}},
{{82, 76, 2},{0, 1, 0},{8, 7, 0}},
{{83, 75, 3},{0, 1, 0},{8, 7, 0}},
{{84, 74, 4},{0, 1, 0},{8, 7, 0}},
{{85, 73, 6},{0, 1, 0},{8, 7, 0}},
{{86, 72, 7},{0, 1, 0},{8, 7, 0}},
{{87, 71, 8},{0, 1, 0},{8, 7, 0}},
{{88, 70, 9},{0, 1, 0},{8, 7, 0}},
{{90, 69, 10},{0, 1, 0},{8, 7, 0}},
{{91, 68, 11},{0, 1, 0},{9, 6, 1}},
{{92, 67, 12},{0, 1, 0},{9, 6, 1}},
{{93, 66, 13},{0, 1, 0},{9, 6, 1}},
{{94, 66, 14},{0, 1, 0},{9, 6, 1}},
{{95, 65, 15},{0, 1, 0},{9, 6, 1}},
{{96, 64, 16},{0, 1, 0},{9, 6, 1}},
{{97, 63, 17},{0, 1, 0},{9, 6, 1}},
{{98, 62, 18},{0, 1, 0},{9, 6, 1}},
{{99, 61, 19},{0, 1, 0},{9, 6, 1}},
{{100, 60, 20},{0, 1, 0},{10, 6, 2}},
{{102, 59, 21},{0, 1, 0},{10, 6, 2}},
{{103, 58, 22},{0, 1, 0},{10, 5, 2}},
{{104, 57, 23},{0, 1, 0},{10, 5, 2}},
{{105, 56, 25},{0, 1, 0},{10, 5, 2}},
{{106, 55, 26},{0, 1, 0},{10, 5, 2}},
{{107, 55, 27},{0, 1, 0},{10, 5, 2}},
{{109, 54, 28},{0, 1, 0},{10, 5, 2}},
{{110, 53, 29},{0, 1, 0},{10, 5, 2}},
{{111, 52, 30},{0, 1, 0},{11, 5, 2}},
{{112, 51, 31},{0, 1, 0},{11, 5, 3}},
{{113, 50, 32},{0, 1, 0},{11, 5, 3}},
{{115, 49, 33},{0, 1, 0},{11, 5, 3}},
{{116, 48, 34},{0, 1, 0},{11, 4, 3}},
{{117, 47, 35},{0, 1, 0},{11, 4, 3}},
{{118, 47, 36},{0, 1, 0},{11, 4, 3}},
{{120, 46, 38},{0, 1, 0},{11, 4, 3}},
{{121, 45, 39},{0, 1, 0},{12, 4, 3}},
{{122, 44, 40},{0, 1, 0},{12, 4, 3}},
{{124, 43, 41},{0, 1, 0},{12, 4, 4}},
{{125, 42, 42},{0, 1, 0},{12, 4, 4}},
{{126, 41, 43},{0, 1, 0},{12, 4, 4}},
{{128, 40, 44},{0, 1, 0},{12, 4, 4}},
{{129, 40, 45},{0, 1, 0},{12, 4, 4}},
{{130, 39, 46},{0, 1, 0},{13, 3, 4}},
{{132, 38, 48},{0, 1, 0},{13, 3, 4}},
{{133, 37, 49},{0, 1, 0},{13, 3, 4}},
{{134, 36, 50},{0, 1, 0},{13, 3, 4}},
{{136, 35, 51},{0, 1, 0},{13, 3, 5}},
{{137, 35, 52},{0, 1, 0},{13, 3, 5}},
{{139, 34, 53},{0, 1, 0},{13, 3, 5}},
{{140, 33, 55},{0, 1, 0},{14, 3, 5}},
{{142, 32, 56},{0, 1, 0},{14, 3, 5}},
{{143, 31, 57},{0, 1, 0},{14, 3, 5}},
{{145, 30, 58},{0, 1, 0},{14, 3, 5}},
{{146, 30, 59},{0, 1, 0},{14, 3, 5}},
{{148, 29, 60},{0, 1, 0},{14, 2, 6}},
{{149, 28, 62},{0, 1, 0},{14, 2, 6}},
{{151, 27, 63},{0, 1, 0},{15, 2, 6}},
{{153, 26, 64},{0, 1, 0},{15, 2, 6}},
{{154, 25, 65},{0, 1, 0},{15, 2, 6}},
{{156, 25, 66},{0, 1, 0},{15, 2, 6}},
{{158, 24, 68},{0, 1, 0},{15, 2, 6}},
{{159, 23, 69},{0, 1, 0},{15, 2, 6}},
{{161, 22, 70},{0, 1, 0},{16, 2, 7}},
{{163, 21, 71},{0, 1, 0},{16, 2, 7}},
{{164, 20, 73},{0, 1, 0},{16, 2, 7}},
{{166, 20, 74},{0, 1, 0},{16, 2, 7}},
{{168, 19, 75},{0, 1, 0},{16, 1, 7}},
{{170, 18, 77},{0, 1, 0},{16, 1, 7}},
{{172, 17, 78},{0, 1, 0},{17, 1, 7}},
{{173, 16, 79},{0, 1, 0},{17, 1, 7}},
{{175, 16, 80},{0, 1, 0},{17, 1, 8}},
{{177, 15, 82},{0, 1, 0},{17, 1, 8}},
{{179, 14, 83},{0, 1, 0},{17, 1, 8}},
{{181, 13, 84},{0, 1, 0},{18, 1, 8}},
{{183, 12, 86},{0, 1, 0},{18, 1, 8}},
{{185, 11, 87},{0, 1, 0},{18, 1, 8}},
{{187, 11, 88},{0, 1, 0},{18, 1, 8}},
{{189, 10, 90},{0, 1, 0},{18, 1, 8}},
{{191, 9, 91},{0, 1, 0},{19, 0, 9}},
{{194, 8, 93},{0, 1, 0},{19, 0, 9}},
{{196, 7, 94},{0, 1, 0},{19, 0, 9}},
{{198, 7, 95},{0, 1, 0},{19, 0, 9}},
{{200, 6, 97},{0, 1, 0},{19, 0, 9}},
{{203, 5, 98},{0, 1, 0},{20, 0, 9}},
{{205, 4, 100},{0, 1, 0},{20, 0, 9}},
{{207, 3, 101},{0, 1, 0},{20, 0, 10}},
{{210, 3, 103},{0, 1, 0},{20, 0, 10}},
{{212, 2, 104},{0, 1, 0},{21, 0, 10}},
{{215, 1, 106},{0, 1, 0},{21, 0, 10}},
{{223, 48, 223},{1, 1, 1},{31, 7, 31}},
{{220, 47, 219},{1, 1, 1},{31, 6, 31}},
{{216, 46, 216},{1, 1, 1},{30, 6, 30}},
{{213, 45, 212},{1, 1, 1},{30, 6, 30}},
{{209, 44, 209},{1, 1, 1},{29, 6, 29}},
{{206, 42, 206},{1, 1, 1},{29, 6, 29}},
{{203, 41, 202},{1, 1, 1},{28, 5, 28}},
{{199, 40, 199},{1, 1, 1},{28, 5, 28}},
{{196, 39, 196},{1, 1, 1},{28, 5, 28}},
{{193, 38, 193},{1, 1, 1},{27, 5, 27}},
{{190, 37, 190},{1, 1, 1},{27, 5, 27}},
{{187, 35, 187},{1, 1, 1},{26, 5, 26}},
{{185, 34, 184},{1, 1, 1},{26, 5, 26}},
{{182, 33, 182},{1, 1, 1},{25, 4, 25}},
{{179, 32, 179},{1, 1, 1},{25, 4, 25}},
{{177, 31, 176},{1, 1, 1},{25, 4, 25}},
{{174, 30, 174},{1, 1, 1},{24, 4, 24}},
{{171, 29, 171},{1, 1, 1},{24, 4, 24}},
{{169, 27, 169},{1, 1, 1},{24, 4, 24}},
{{166, 26, 166},{1, 1, 1},{23, 3, 23}},
{{164, 25, 164},{1, 1, 1},{23, 3, 23}},
{{162, 24, 161},{1, 1, 1},{23, 3, 23}},
{{159, 23, 159},{1, 1, 1},{22, 3, 22}},
{{157, 22, 157},{1, 1, 1},{22, 3, 22}},
{{155, 21, 154},{1, 1, 1},{22, 3, 22}},
{{152, 19, 152},{1, 1, 1},{21, 2, 21}},
{{150, 18, 150},{1, 1, 1},{21, 2, 21}},
{{148, 17, 148},{1, 1, 1},{21, 2, 21}},
{{146, 16, 146},{1, 1, 1},{20, 2, 20}},
{{144, 15, 144},{1, 1, 1},{20, 2, 20}},
{{142, 14, 142},{1, 1, 1},{20, 2, 20}},
{{140, 13, 140},{1, 1, 1},{19, 1, 19}},
{{138, 12, 138},{1, 1, 1},{19, 1, 19}},
{{136, 11, 136},{1, 1, 1},{19, 1, 19}},
{{134, 9, 134},{1, 1, 1},{19, 1, 19}},
{{132, 8, 132},{1, 1, 1},{18, 1, 18}},
{{130, 7, 130},{1, 1, 1},{18, 1, 18}},
{{128, 6, 128},{1, 1, 1},{18, 1, 18}},
{{126, 5, 126},{1, 1, 1},{18, 0, 18}},
{{125, 4, 124},{1, 1, 1},{17, 0, 17}},
{{123, 3, 123},{1, 1, 1},{17, 0, 17}},
{{121, 2, 121},{1, 1, 1},{17, 0, 17}},
{{119, 1, 119},{1, 1, 1},{17, 0, 17}},
{{117, 0, 117},{1, 1, 1},{16, 0, 16}},
{{116, 1, 116},{1, 0, 1},{16, 0, 16}},
{{114, 2, 114},{1, 0, 1},{16, 0, 16}},
{{112, 3, 112},{1, 0, 1},{16, 0, 16}},
{{111, 4, 111},{1, 0, 1},{15, 0, 15}},
{{109, 5, 109},{1, 0, 1},{15, 0, 15}},
{{107, 6, 107},{1, 0, 1},{15, 0, 15}},
{{106, 7, 106},{1, 0, 1},{15, 1, 15}},
{{104, 8, 104},{1, 0, 1},{14, 1, 14}},
{{103, 9, 102},{1, 0, 1},{14, 1, 14}},
{{101, 11, 101},{1, 0, 1},{14, 1, 14}},
{{100, 12, 99},{1, 0, 1},{14, 1, 14}},
{{98, 13, 98},{1, 0, 1},{14, 1, 14}},
{{96, 14, 96},{1, 0, 1},{13, 1, 13}},
{{95, 15, 95},{1, 0, 1},{13, 2, 13}},
{{93, 16, 93},{1, 0, 1},{13, 2, 13}},
{{92, 17, 92},{1, 0, 1},{13, 2, 13}},
{{90, 18, 90},{1, 0, 1},{12, 2, 12}},
{{89, 19, 89},{1, 0, 1},{12, 2, 12}},
{{88, 21, 87},{1, 0, 1},{12, 2, 12}},
{{86, 22, 86},{1, 0, 1},{12, 3, 12}},
{{85, 23, 85},{1, 0, 1},{12, 3, 12}},
{{83, 24, 83},{1, 0, 1},{11, 3, 11}},
{{82, 25, 82},{1, 0, 1},{11, 3, 11}},
{{81, 26, 80},{1, 0, 1},{11, 3, 11}},
{{79, 27, 79},{1, 0, 1},{11, 3, 11}},
{{78, 29, 78},{1, 0, 1},{11, 4, 11}},
{{76, 30, 76},{1, 0, 1},{10, 4, 10}},
{{75, 31, 75},{1, 0, 1},{10, 4, 10}},
{{74, 32, 74},{1, 0, 1},{10, 4, 10}},
{{72, 33, 72},{1, 0, 1},{10, 4, 10}},
{{71, 34, 71},{1, 0, 1},{10, 4, 10}},
{{70, 35, 70},{1, 0, 1},{10, 4, 10}},
{{68, 37, 68},{1, 0, 1},{9, 5, 9}},
{{67, 38, 67},{1, 0, 1},{9, 5, 9}},
{{66, 39, 66},{1, 0, 1},{9, 5, 9}},
{{64, 40, 64},{1, 0, 1},{9, 5, 9}},
{{63, 41, 63},{1, 0, 1},{9, 5, 9}},
{{62, 42, 62},{1, 0, 1},{8, 5, 8}},
{{61, 44, 61},{1, 0, 1},{8, 6, 8}},
{{59, 45, 59},{1, 0, 1},{8, 6, 8}},
{{58, 46, 58},{1, 0, 1},{8, 6, 8}},
{{57, 47, 57},{1, 0, 1},{8, 6, 8}},
{{56, 48, 56},{1, 0, 1},{8, 6, 8}},
{{54, 50, 54},{1, 0, 1},{7, 7, 7}},
{{53, 51, 53},{1, 0, 1},{7, 7, 7}},
{{52, 52, 52},{1, 0, 1},{7, 7, 7}},
{{51, 53, 51},{1, 0, 1},{7, 7, 7}},
{{50, 55, 49},{1, 0, 1},{7, 7, 7}},
{{48, 56, 48},{1, 0, 1},{6, 7, 6}},
{{47, 57, 47},{1, 0, 1},{6, 8, 6}},
{{46, 58, 46},{1, 0, 1},{6, 8, 6}},
{{45, 60, 45},{1, 0, 1},{6, 8, 6}},
{{44, 61, 43},{1, 0, 1},{6, 8, 6}},
{{42, 62, 42},{1, 0, 1},{6, 8, 6}},
{{41, 63, 41},{1, 0, 1},{5, 8, 5}},
{{40, 65, 40},{1, 0, 1},{5, 9, 5}},
{{39, 66, 39},{1, 0, 1},{5, 9, 5}},
{{38, 67, 38},{1, 0, 1},{5, 9, 5}},
{{37, 68, 36},{1, 0, 1},{5, 9, 5}},
{{35, 70, 35},{1, 0, 1},{5, 9, 5}},
{{34, 71, 34},{1, 0, 1},{4, 10, 4}},
{{33, 72, 33},{1, 0, 1},{4, 10, 4}},
{{32, 74, 32},{1, 0, 1},{4, 10, 4}},
{{31, 75, 31},{1, 0, 1},{4, 10, 4}},
{{30, 76, 29},{1, 0, 1},{4, 10, 4}},
{{28, 78, 28},{1, 0, 1},{4, 10, 4}},
{{27, 79, 27},{1, 0, 1},{4, 11, 3}},
{{26, 81, 26},{1, 0, 1},{3, 11, 3}},
{{25, 82, 25},{1, 0, 1},{3, 11, 3}},
{{24, 83, 24},{1, 0, 1},{3, 11, 3}},
{{23, 85, 23},{1, 0, 1},{3, 11, 3}},
{{22, 86, 22},{1, 0, 1},{3, 12, 3}},
{{21, 88, 20},{1, 0, 1},{3, 12, 3}},
{{19, 89, 19},{1, 0, 1},{2, 12, 2}},
{{18, 91, 18},{1, 0, 1},{2, 12, 2}},
{{17, 92, 17},{1, 0, 1},{2, 12, 2}},
{{16, 94, 16},{1, 0, 1},{2, 13, 2}},
{{15, 95, 15},{1, 0, 1},{2, 13, 2}},
{{14, 97, 14},{1, 0, 1},{2, 13, 2}},
{{13, 98, 13},{1, 0, 1},{1, 13, 1}},
{{12, 100, 11},{1, 0, 1},{1, 14, 1}},
{{10, 101, 10},{1, 0, 1},{1, 14, 1}},
{{9, 103, 9},{1, 0, 1},{1, 14, 1}},
{{8, 104, 8},{1, 0, 1},{1, 14, 1}},
{{7, 106, 7},{1, 0, 1},{1, 14, 1}},
{{6, 108, 6},{1, 0, 1},{1, 15, 0}},
{{5, 109, 5},{1, 0, 1},{0, 15, 0}},
{{4, 111, 4},{1, 0, 1},{0, 15, 0}},
{{3, 112, 3},{1, 0, 1},{0, 15, 0}},
{{2, 114, 2},{1, 0, 1},{0, 16, 0}},
{{1, 116, 0},{1, 0, 1},{0, 16, 0}},
{{0, 118, 0},{1, 0, 1},{0, 16, 0}},
{{1, 119, 1},{0, 0, 0},{0, 16, 0}},
{{2, 121, 2},{0, 0, 0},{0, 17, 0}},
{{3, 123, 3},{0, 0, 0},{0, 17, 0}},
{{4, 125, 4},{0, 0, 0},{0, 17, 0}},
{{5, 126, 5},{0, 0, 0},{0, 17, 0}},
{{6, 128, 6},{0, 0, 0},{0, 18, 0}},
{{7, 130, 7},{0, 0, 0},{1, 18, 1}},
{{8, 132, 9},{0, 0, 0},{1, 18, 1}},
{{10, 134, 10},{0, 0, 0},{1, 18, 1}},
{{11, 136, 11},{0, 0, 0},{1, 19, 1}},
{{12, 138, 12},{0, 0, 0},{1, 19, 1}},
{{13, 140, 13},{0, 0, 0},{1, 19, 1}},
{{14, 142, 14},{0, 0, 0},{1, 20, 1}},
{{15, 144, 15},{0, 0, 0},{2, 20, 2}},
{{16, 146, 16},{0, 0, 0},{2, 20, 2}},
{{17, 148, 17},{0, 0, 0},{2, 20, 2}},
{{18, 150, 19},{0, 0, 0},{2, 21, 2}},
{{20, 153, 20},{0, 0, 0},{2, 21, 2}},
{{21, 155, 21},{0, 0, 0},{2, 21, 2}},
{{22, 157, 22},{0, 0, 0},{3, 22, 3}},
{{23, 159, 23},{0, 0, 0},{3, 22, 3}},
{{24, 162, 24},{0, 0, 0},{3, 22, 3}},
{{25, 164, 25},{0, 0, 0},{3, 23, 3}},
{{26, 167, 26},{0, 0, 0},{3, 23, 3}},
{{27, 169, 28},{0, 0, 0},{3, 23, 3}},
{{29, 172, 29},{0, 0, 0},{4, 24, 4}},
{{30, 174, 30},{0, 0, 0},{4, 24, 4}},
{{31, 177, 31},{0, 0, 0},{4, 24, 4}},
{{32, 179, 32},{0, 0, 0},{4, 25, 4}},
{{33, 182, 33},{0, 0, 0},{4, 25, 4}},
{{34, 185, 34},{0, 0, 0},{4, 26, 4}},
{{36, 188, 36},{0, 0, 0},{5, 26, 5}},
{{37, 191, 37},{0, 0, 0},{5, 26, 5}},
{{38, 194, 38},{0, 0, 0},{5, 27, 5}},
{{39, 197, 39},{0, 0, 0},{5, 27, 5}},
{{40, 200, 40},{0, 0, 0},{5, 28, 5}},
{{41, 203, 42},{0, 0, 0},{5, 28, 5}},
{{43, 206, 43},{0, 0, 0},{6, 28, 6}},
{{44, 209, 44},{0, 0, 0},{6, 29, 6}},
{{45, 213, 45},{0, 0, 0},{6, 29, 6}},
{{46, 216, 46},{0, 0, 0},{6, 30, 6}},
{{47, 220, 47},{0, 0, 0},{6, 30, 6}},
{{49, 224, 49},{0, 0, 0},{6, 31, 6}},
{{50, 227, 50},{0, 0, 0},{7, 31, 7}},
{{49, 224, 49},{1, 1, 1},{7, 31, 7}},
{{47, 220, 47},{1, 1, 1},{6, 31, 6}},
{{46, 216, 46},{1, 1, 1},{6, 30, 6}},
{{45, 213, 45},{1, 1, 1},{6, 30, 6}},
{{44, 209, 44},{1, 1, 1},{6, 29, 6}},
{{43, 206, 43},{1, 1, 1},{6, 29, 6}},
{{41, 203, 42},{1, 1, 1},{6, 28, 6}},
{{40, 200, 40},{1, 1, 1},{5, 28, 5}},
{{39, 197, 39},{1, 1, 1},{5, 28, 5}},
{{38, 194, 38},{1, 1, 1},{5, 27, 5}},
{{37, 191, 37},{1, 1, 1},{5, 27, 5}},
{{36, 188, 36},{1, 1, 1},{5, 26, 5}},
{{34, 185, 34},{1, 1, 1},{5, 26, 5}},
{{33, 182, 33},{1, 1, 1},{4, 26, 4}},
{{32, 179, 32},{1, 1, 1},{4, 25, 4}},
{{31, 177, 31},{1, 1, 1},{4, 25, 4}},
{{30, 174, 30},{1, 1, 1},{4, 24, 4}},
{{29, 172, 29},{1, 1, 1},{4, 24, 4}},
{{27, 169, 28},{1, 1, 1},{4, 24, 4}},
{{26, 167, 26},{1, 1, 1},{3, 23, 3}},
{{25, 164, 25},{1, 1, 1},{3, 23, 3}},
{{24, 162, 24},{1, 1, 1},{3, 23, 3}},
{{23, 159, 23},{1, 1, 1},{3, 22, 3}},
{{22, 157, 22},{1, 1, 1},{3, 22, 3}},
{{21, 155, 21},{1, 1, 1},{3, 22, 3}},
{{20, 153, 20},{1, 1, 1},{2, 21, 2}},
{{18, 150, 19},{1, 1, 1},{2, 21, 2}},
{{17, 148, 17},{1, 1, 1},{2, 21, 2}},
{{16, 146, 16},{1, 1, 1},{2, 20, 2}},
{{15, 144, 15},{1, 1, 1},{2, 20, 2}},
{{14, 142, 14},{1, 1, 1},{2, 20, 2}},
{{13, 140, 13},{1, 1, 1},{1, 20, 1}},
{{12, 138, 12},{1, 1, 1},{1, 19, 1}},
{{11, 136, 11},{1, 1, 1},{1, 19, 1}},
{{10, 134, 10},{1, 1, 1},{1, 19, 1}},
{{8, 132, 9},{1, 1, 1},{1, 18, 1}},
{{7, 130, 7},{1, 1, 1},{1, 18, 1}},
{{6, 128, 6},{1, 1, 1},{1, 18, 1}},
{{5, 126, 5},{1, 1, 1},{0, 18, 0}},
{{4, 125, 4},{1, 1, 1},{0, 17, 0}},
{{3, 123, 3},{1, 1, 1},{0, 17, 0}},
{{2, 121, 2},{1, 1, 1},{0, 17, 0}},
{{1, 119, 1},{1, 1, 1},{0, 17, 0}},
{{0, 118, 0},{1, 1, 1},{0, 16, 0}},
{{1, 116, 0},{0, 1, 0},{0, 16, 0}},
{{2, 114, 2},{0, 1, 0},{0, 16, 0}},
{{3, 112, 3},{0, 1, 0},{0, 16, 0}},
{{4, 111, 4},{0, 1, 0},{0, 15, 0}},
{{5, 109, 5},{0, 1, 0},{0, 15, 0}},
{{6, 108, 6},{0, 1, 0},{0, 15, 0}},
{{7, 106, 7},{0, 1, 0},{1, 15, 0}},
{{8, 104, 8},{0, 1, 0},{1, 14, 1}},
{{9, 103, 9},{0, 1, 0},{1, 14, 1}},
{{10, 101, 10},{0, 1, 0},{1, 14, 1}},
{{12, 100, 11},{0, 1, 0},{1, 14, 1}},
{{13, 98, 13},{0, 1, 0},{1, 14, 1}},
{{14, 97, 14},{0, 1, 0},{1, 13, 1}},
{{15, 95, 15},{0, 1, 0},{2, 13, 2}},
{{16, 94, 16},{0, 1, 0},{2, 13, 2}},
{{17, 92, 17},{0, 1, 0},{2, 13, 2}},
{{18, 91, 18},{0, 1, 0},{2, 12, 2}},
{{19, 89, 19},{0, 1, 0},{2, 12, 2}},
{{21, 88, 20},{0, 1, 0},{2, 12, 2}},
{{22, 86, 22},{0, 1, 0},{3, 12, 3}},
{{23, 85, 23},{0, 1, 0},{3, 12, 3}},
{{24, 83, 24},{0, 1, 0},{3, 11, 3}},
{{25, 82, 25},{0, 1, 0},{3, 11, 3}},
{{26, 81, 26},{0, 1, 0},{3, 11, 3}},
{{27, 79, 27},{0, 1, 0},{3, 11, 3}},
{{28, 78, 28},{0, 1, 0},{4, 11, 3}},
{{30, 76, 29},{0, 1, 0},{4, 10, 4}},
{{31, 75, 31},{0, 1, 0},{4, 10, 4}},
{{32, 74, 32},{0, 1, 0},{4, 10, 4}},
{{33, 72, 33},{0, 1, 0},{4, 10, 4}},
{{34, 71, 34},{0, 1, 0},{4, 10, 4}},
{{35, 70, 35},{0, 1, 0},{4, 10, 4}},
{{37, 68, 36},{0, 1, 0},{5, 9, 5}},
{{38, 67, 38},{0, 1, 0},{5, 9, 5}},
{{39, 66, 39},{0, 1, 0},{5, 9, 5}},
{{40, 65, 40},{0, 1, 0},{5, 9, 5}},
{{41, 63, 41},{0, 1, 0},{5, 9, 5}},
{{42, 62, 42},{0, 1, 0},{5, 8, 5}},
{{44, 61, 43},{0, 1, 0},{6, 8, 6}},
{{45, 60, 45},{0, 1, 0},{6, 8, 6}},
{{46, 58, 46},{0, 1, 0},{6, 8, 6}},
{{47, 57, 47},{0, 1, 0},{6, 8, 6}},
{{48, 56, 48},{0, 1, 0},{6, 8, 6}},
{{50, 55, 49},{0, 1, 0},{6, 7, 6}},
{{51, 53, 51},{0, 1, 0},{7, 7, 7}},
{{52, 52, 52},{0, 1, 0},{7, 7, 7}},
{{200, 200, 200},{1, 1, 1},{600, 600, 600}}};



extern block_buff_t block_buffer;

void Test_Block_0()
{
    block_t new_block;
    new_block.flag = block_ready;
    for (uint8_t i=0;i<3;i++)
    {
        new_block.dir[i] = carriage_DOWN;
        new_block.maximum_freq[i] = 500;
        new_block.step[i] = 20;
    }
		//new_block.maximum_freq[1] = 100;
		//new_block.step[0] = 400;
    Block_Buff_Write(new_block, &block_buffer);
}


void Test_Block_1()
{
    block_t new_block;
    new_block.flag = block_ready;
    for (uint8_t i=0;i<3;i++)
    {
        new_block.dir[i] = carriage_UP;
        new_block.maximum_freq[i] = 500;
        new_block.step[i] = 2000;
    }
		//new_block.maximum_freq[1] = 100;
		//new_block.step[1] = 400;
    Block_Buff_Write(new_block, &block_buffer);
}

void Test_Block_2()
{
    block_t new_block;
    new_block.flag = block_ready;
    for (uint8_t i=0;i<3;i++)
    {
        new_block.dir[i] = carriage_UP;
        new_block.maximum_freq[i] = 15;
        new_block.step[i] = 102;
    }
    Block_Buff_Write(new_block, &block_buffer);
}

void Test_Block_3()
{
    block_t new_block;
    new_block.flag = block_ready;
    for (uint8_t i=0;i<3;i++)
    {
        new_block.dir[i] = carriage_DOWN;
    }

    new_block.step[0] = 1200;
    new_block.step[1] = 2700;
    new_block.step[2] = 2700;
    new_block.maximum_freq[0] = 60;
    new_block.maximum_freq[1] = 150;
    new_block.maximum_freq[2] = 150;

    Block_Buff_Write(new_block, &block_buffer);
}




