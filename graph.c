#include <stdlib.h>
#include "graph.h"

void push(node_t * head, cells cell) {
    node_t * current = head;
    while (current->next != NULL) {
        current = current->next;
    }

    current->next = malloc(sizeof(node_t));
    current->next->cell = cell;
    current->next->next = NULL;
}

void remove_last(node_t * head) {
    
    if (head->next == NULL) 
        free(head);

    node_t * current = head;
    while (current->next->next != NULL) {
        current = current->next;
    }

    free(current->next);
    current->next = NULL;

}