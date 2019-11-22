/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Definitions of LRU tag store.
 */

#include <string>

#include "base/intmath.hh"
#include "debug/CacheRepl.hh"
#include "mem/cache/tags/cacheset.hh"
#include "mem/cache/tags/lru.hh"
#include "mem/cache/base.hh"
#include "sim/core.hh"

using namespace std;

// create and initialize a LRU/MRU cache structure
/*LRU::LRU(unsigned _numSets, unsigned _blkSize, unsigned _assoc,
         unsigned _hit_latency, unsigned _victim_addition)
    : numSets(_numSets), blkSize(_blkSize), assoc(_assoc),
      hitLatency(_hit_latency),victim_addition(_victim_addition)
*/
LRU::LRU(unsigned _numSets, unsigned _blkSize, unsigned _assoc,
         unsigned _hit_latency)
    : numSets(_numSets), blkSize(_blkSize), assoc(_assoc),
      hitLatency(_hit_latency)
{
    // Check parameters
    if (blkSize < 4 || !isPowerOf2(blkSize)) {
        fatal("Block size must be at least 4 and a power of 2");
    }
    if (numSets <= 0 || !isPowerOf2(numSets)) {
        fatal("# of sets must be non-zero and a power of 2");
    }
    if (assoc <= 0) {
        fatal("associativity must be greater than zero");
    }
    if (hitLatency <= 0) {
        fatal("access latency must be greater than zero");
    }

    blkMask = blkSize - 1;
    setShift = floorLog2(blkSize);
    setMask = numSets - 1;
    tagShift = setShift + floorLog2(numSets);
    warmedUp = false;
    /** @todo Make warmup percentage a parameter. */
    warmupBound = numSets * assoc;
    victim_addition =true;
    //printf(" in here in here in here \n");
	unsigned blkIndex = 0;       // index into blks array
/////////// Modifications for victim cache //////////////////////////
	if(victim_addition)
	{
		num_victim_set=1;
		num_victim_size=8;
	printf("creating victim cache \n");
        victim_sets= new CacheSet[num_victim_set];
        blks = new BlkType[num_victim_size];
        victim_data_blks = new uint8_t[num_victim_set*num_victim_size*blkSize];

    for (unsigned i = 0; i < num_victim_set; ++i) {
        victim_sets[i].assoc = num_victim_size;

        victim_sets[i].blks = new BlkType*[num_victim_size];

        // link in the data blocks
        for (unsigned j = 0; j < num_victim_size; ++j) {
            // locate next cache block
            BlkType *blk = &blks[blkIndex];
            blk->data = &victim_data_blks[blkSize*blkIndex];
            ++blkIndex;

            // invalidate new cache block
            blk->invalidate();

            //EGH Fix Me : do we need to initialize blk?

            // Setting the tag to j is just to prevent long chains in the hash
            // table; won't matter because the block is invalid
            blk->tag = j;
            blk->whenReady = 0;
            blk->isTouched = false;
            blk->size = blkSize;
            victim_sets[0].blks[j]=blk;
            blk->set = i;
        }
    }

}
///////////////////////////////////////////////////////////////
	
    sets = new CacheSet[numSets];
    blks = new BlkType[numSets * assoc];
    // allocate data storage in one big chunk
    numBlocks = numSets * assoc;
    dataBlks = new uint8_t[numBlocks * blkSize];

    blkIndex = 0;       // index into blks array
    for (unsigned i = 0; i < numSets; ++i) {
        sets[i].assoc = assoc;

        sets[i].blks = new BlkType*[assoc];

        // link in the data blocks
        for (unsigned j = 0; j < assoc; ++j) {
            // locate next cache block
            BlkType *blk = &blks[blkIndex];
            blk->data = &dataBlks[blkSize*blkIndex];
            ++blkIndex;

            // invalidate new cache block
            blk->invalidate();

            //EGH Fix Me : do we need to initialize blk?

            // Setting the tag to j is just to prevent long chains in the hash
            // table; won't matter because the block is invalid
            blk->tag = j;
            blk->whenReady = 0;
            blk->isTouched = false;
            blk->size = blkSize;
            sets[i].blks[j]=blk;
            blk->set = i;
        }
    }
}

LRU::~LRU()
{
    delete [] dataBlks;
    delete [] blks;
    delete [] sets;
}

LRU::BlkType*
LRU::accessBlock(Addr addr, int &lat, int master_id)
{
    printf("in access blk \n");
    Addr tag = extractTag(addr);
    unsigned set = extractSet(addr);
    BlkType *blk = sets[set].findBlk(tag);
    lat = hitLatency;
    if (blk != NULL) {
        // move this block to head of the MRU list
        //printf("move to head 1 \n");
        sets[set].moveToHead(blk);
        DPRINTF(CacheRepl, "set %x: moving blk %x to MRU\n",
                set, regenerateBlkAddr(tag, set));
        if (blk->whenReady > curTick()
            && blk->whenReady - curTick() > hitLatency) {
            lat = blk->whenReady - curTick();
        }
        blk->refCount += 1;
    }
   if((blk== NULL) &&(victim_addition==1))
   {
		//printf("blk == NULL ");
    		printf("blk not found searching in victim \n");
		for (int i = 0; i < 8; ++i)
		{
			if ((victim_sets[0].blks[i]->tag == addr) && (((victim_sets[0].blks[i]->status) & 0x01) !=0))
			{
				printf("Found in victim cache \n");
				temp = new BlkType[1];
				printf("1 \n");
				temp = victim_sets[0].blks[i];
				printf("2 \n");
				victim_sets[0].blks[i] = sets[set].blks[assoc-1];
				printf("3 \n");
			 	sets[set].blks[assoc-1]=temp;
				printf("4 \n");
				BlkType *blk2 = sets[set].blks[assoc-1];
				printf("5 \n");
				blk2->tag=extractTag(addr);
				printf("6 \n");
				//printf("Attempting to move victim block to head \n");	
        			//printf("move to head 2 \n");
			 	blk=victim_sets[0].blks[i];
				printf("7 \n");
				victim_sets[0].moveToHead(blk);
				printf("8 \n");
				//printf(" move victim block to head done \n");	
				blk=sets[set].blks[assoc-1];
				printf("9 \n");
        			//printf("move to head 3 \n");
				sets[set].moveToHead(blk);
				printf("10 \n");
			}
		}
        //lat = hitLatency+1;
   }
printf("check if completing \n");
    return blk;
}


LRU::BlkType*
LRU::findBlock(Addr addr) const
{
    Addr tag = extractTag(addr);
    unsigned set = extractSet(addr);
    BlkType *blk = sets[set].findBlk(tag);
    return blk;
}

LRU::BlkType*
LRU::findVictim(Addr addr, PacketList &writebacks)
{
    unsigned set = extractSet(addr);
    // grab a replacement candidate
    BlkType *blk2 = sets[set].blks[assoc-1];
	if ((blk2->isValid())&&(victim_addition==1)) 
	{
    		printf("find victim \n");
    		//printf(" blk -> data in actual cache = %d \n",*(blk2->data));
		//BlkType *blk5 = victim_sets[0].blks[7];
		temp = new BlkType[1];
		temp = victim_sets[0].blks[7];
		//printf(" blk -> data in victim cache initially = %d \n",*(blk5->data));
		victim_sets[0].blks[7] = sets[set].blks[assoc-1];
		BlkType *blk3 = victim_sets[0].blks[7];
		blk3->tag=regenerateBlkAddr(blk2->tag,set);
		printf(" intial address = %lu blk tag is = %lu \n",addr,blk3->tag);
		//blk3->tag=addr;
		//printf("in move to head 4 \n");	
		//printf("assoc of victim cache = %d ",victim_sets[0].assoc);
		victim_sets[0].moveToHead(blk3);
        }
    //BlkType *blk4 = sets[set].blks[assoc-1];
    //printf(" Data to be evicted in cache same guy as before  = %d \n",*(blk4->data));
    return blk2;
}

void
LRU::insertBlock(Addr addr, BlkType *blk, int master_id)
{
    if (!blk->isTouched) {
        tagsInUse++;
        blk->isTouched = true;
        if (!warmedUp && tagsInUse.value() >= warmupBound) {
            warmedUp = true;
            printf("The cache has warmed up --------------- \n");
            warmupCycle = curTick();
        }
    }

    // If we're replacing a block that was previously valid update
    // stats for it. This can't be done in findBlock() because a
    // found block might not actually be replaced there if the
    // coherence protocol says it can't be.
    if (blk->isValid()) {
        replacements[0]++;
        totalRefs += blk->refCount;
        ++sampledRefs;
        blk->refCount = 0;

        // deal with evicted block
        assert(blk->srcMasterId < cache->system->maxMasters());
        occupancies[blk->srcMasterId]--; //Reducing cache occupancy

        blk->invalidate();
    }

    blk->isTouched = true;
    // Set tag for new block.  Caller is responsible for setting status.
    blk->tag = extractTag(addr);

    // deal with what we are bringing in
    assert(master_id < cache->system->maxMasters());
    occupancies[master_id]++;
    blk->srcMasterId = master_id;

    unsigned set = extractSet(addr);
    //printf("in move to head 5 \n");
    sets[set].moveToHead(blk); // Change this line for Insertion policy
}

void
LRU::invalidate(BlkType *blk)
{
    assert(blk);
    assert(blk->isValid());
    tagsInUse--;
    assert(blk->srcMasterId < cache->system->maxMasters());
    occupancies[blk->srcMasterId]--;
    blk->srcMasterId = Request::invldMasterId;

    // should be evicted before valid blocks
    unsigned set = blk->set;
    sets[set].moveToTail(blk);
}

void
LRU::clearLocks()
{
    for (int i = 0; i < numBlocks; i++){
        blks[i].clearLoadLocks();
    }
}

void
LRU::cleanupRefs()
{
    for (unsigned i = 0; i < numSets*assoc; ++i) {
        if (blks[i].isValid()) {
            totalRefs += blks[i].refCount;
            ++sampledRefs;
        }
    }
}
