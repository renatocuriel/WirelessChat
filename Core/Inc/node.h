#ifndef INC_NODE_H_
#define INC_NODE_H_

#include "main.h"

#define TIME_TO_LIVE 110

typedef struct {
	char name[21];
	uint8_t address;
	uint8_t timer;
	uint8_t online;
	struct connectionNode *next;
} connectionNode;

void addNode(uint8_t address, char *name, uint8_t len);
connectionNode* getNode(uint8_t address);
void removeOfflineNodes(void);
void refreshNode(uint8_t address);
void decrementTimers(void);
int inList(uint8_t address);
void printList(void);
void freeList(void);

#endif /* INC_NODE_H_ */
