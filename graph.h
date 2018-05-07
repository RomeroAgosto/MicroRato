#ifndef __GRAPH_H
#define __GRAPH_H

#ifdef __cplusplus
extern "C" {
#endif

typedef union
{
	struct
	{ 
		int idCell;
		int cost_startCell;
		int pos_x;
		int pos_y;
		char visited;
	};
}cells;

typedef struct node {
    cells cell;
    struct node * next;
} node_t;


void push(node_t * head, cells cell);
void remove_last(node_t * head);

#ifdef __cplusplus
}
#endif

#endiF