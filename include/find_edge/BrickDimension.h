#ifndef __BRICKDIMENSION_H__
#define __BRICKDIMENSION_H__

#define CLASSES 11 // numero classi dei brick
#define EDGE_BASE 0.03 // lato base dimensione su gazebo
#define MULTI_1 1 //moltiplicatori del lato base
#define MULTI_2 2
#define MULTI_3 3
#define MULTI_4 4

struct dimensions{
    float x; // dimensione su x
    float y; // dimensione su y
    float z; // dimensione su z -> altezza
};

const struct dimensions BRICKS[CLASSES]={
    {.x=EDGE_BASE*MULTI_1, .y=EDGE_BASE*MULTI_1, .z=EDGE_BASE*MULTI_2}, {.x=EDGE_BASE*MULTI_1, .y=EDGE_BASE*MULTI_2, .z=EDGE_BASE*MULTI_1},
    {.x=EDGE_BASE*MULTI_1, .y=EDGE_BASE*MULTI_2, .z=EDGE_BASE*MULTI_2}, {.x=EDGE_BASE*MULTI_1, .y=EDGE_BASE*MULTI_2, .z=EDGE_BASE*MULTI_2},
    {.x=EDGE_BASE*MULTI_1, .y=EDGE_BASE*MULTI_2, .z=EDGE_BASE*MULTI_2}, {.x=EDGE_BASE*MULTI_1, .y=EDGE_BASE*MULTI_3, .z=EDGE_BASE*MULTI_2},
    {.x=EDGE_BASE*MULTI_1, .y=EDGE_BASE*MULTI_3, .z=EDGE_BASE*MULTI_2}, {.x=EDGE_BASE*MULTI_1, .y=EDGE_BASE*MULTI_4, .z=EDGE_BASE*MULTI_1},
    {.x=EDGE_BASE*MULTI_1, .y=EDGE_BASE*MULTI_4, .z=EDGE_BASE*MULTI_2}, {.x=EDGE_BASE*MULTI_2, .y=EDGE_BASE*MULTI_2, .z=EDGE_BASE*MULTI_2},
    {.x=EDGE_BASE*MULTI_2, .y=EDGE_BASE*MULTI_2, .z=EDGE_BASE*MULTI_2}};
/* classi ordinate come su yolo 
 ex) se X1-> EDGE_BASE * MULTI_1 se X2-> EDGE_BASE*MULTI_2
'X1-Y1-Z2', 'X1-Y2-Z1', 
'X1-Y2-Z2','X1-Y2-Z2, 
'X1-Y2-Z2, 'X1-Y3-Z2',
'X1-Y3-Z2,  'X1-Y4-Z1',
'X1-Y4-Z2', 'X2-Y2-Z2',
 'X2-Y2-Z2-FILLET
*/
#endif
