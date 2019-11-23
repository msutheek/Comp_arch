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
	int no_victim;
   if(victim_addition==1)
   {
	    no_victim = 8;
		victim_cache = new CacheSet[1];
    }
    sets = new CacheSet[numSets];
    blks = new BlkType[(numSets * assoc) + no_victim];

    // allocate data storage in one big chunk
    numBlocks = (numSets * assoc)+no_victim;
    dataBlks = new uint8_t[numBlocks * blkSize];

    unsigned blkIndex = 0;       // index into blks array
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
    if(victim_addition == 1)
    {
	    victim_cache[0].assoc = no_victim;
	    victim_cache[0].blks = new BlkType*[no_victim];
	    for (unsigned j = 0; j < no_victim; ++j) {
		    // locate next cache block
		    BlkType *blk = &blks[blkIndex];
		    blk->data = &dataBlks[blkSize*blkIndex];
		    ++blkIndex;
		    // invalidate new cache block
		    blk->invalidate();
		    blk->tag = j;
		    blk->whenReady = 0;
		    blk->isTouched = false;
		    blk->size = blkSize;
		    victim_cache[0].blks[j]=blk;
		    blk->set = 0;
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
	int no_victim=8;	
	bool check = false;
	///printf("in access blk \n");
    Addr tag = extractTag(addr);
    unsigned set = extractSet(addr);
    BlkType *blk = sets[set].findBlk(tag);
    lat = hitLatency;
	if((blk == NULL) && (victim_addition == 1)){
	printf("Checking in victim cache\n");
	for (int i = 0; i < no_victim; ++i) {
		if ((victim_cache[0].blks[i]->tag == tag) && (victim_cache[0].blks[i]->set == set) && (((victim_cache[0].blks[i]->status) & 0x01) !=0)){
		check = true;
		printf("Found in victim cache \n");
		blk = victim_cache[0].blks[i];
		victim_cache[0].blks[i] = sets[set].blks[assoc-1];
		printf("associativity of victim cache is %d \n",victim_cache[0].assoc);
		victim_cache[0].moveToHead(victim_cache[0].blks[i]);
		sets[set].blks[assoc-1] = blk;
		//blk = sets[set].blks[assoc-1];
		}
		if(check) break;
	}

    if (blk != NULL) {
        // move this block to head of the MRU list
        //printf("move to head 1 \n");
        //if(assoc!=1){
		sets[set].moveToHead(blk);//}
        DPRINTF(CacheRepl, "set %x: moving blk %x to MRU\n",
                set, regenerateBlkAddr(tag, set));
        if (blk->whenReady > curTick()
            && blk->whenReady - curTick() > hitLatency) {
            lat = blk->whenReady - curTick();
        }
        blk->refCount += 1;
    }
}

    return blk;
}


LRU::BlkType*
LRU::findBlock(Addr addr) const
{
    int no_victim=8;
	bool check=false;
	Addr tag = extractTag(addr);
    unsigned set = extractSet(addr);
    BlkType *blk = sets[set].findBlk(tag);
	if((blk == NULL) && (victim_addition == 1)){
	for (int i = 0; i < no_victim; ++i) {
		if ((victim_cache[0].blks[i]->tag == tag) && (victim_cache[0].blks[i]->set == set) && (((victim_cache[0].blks[i]->status) & 0x01) !=0)){
		check = true;
		//printf("Found in victim cache \n");
		blk = victim_cache[0].blks[i];
		//blk = sets[set].blks[assoc-1];
		}
		if(check) break;
	}
}
    return blk;
}

LRU::BlkType*
LRU::findVictim(Addr addr, PacketList &writebacks)
{
	int no_victim=8;
    unsigned set = extractSet(addr);
    // grab a replacement candidate
     BlkType *blk = sets[set].blks[assoc-1];
	if(victim_addition == 1){
	BlkType *tmp = victim_cache[0].blks[no_victim-1];
	victim_cache[0].blks[no_victim-1] = blk;
	victim_cache[0].blks[no_victim-1]->tag = blk->tag;
	victim_cache[0].blks[no_victim-1]->set = set;
	victim_cache[0].moveToHead(blk);
	sets[set].blks[assoc-1] = tmp;
	blk=tmp;
	}
     

if (blk->isValid()) { 
	DPRINTF(CacheRepl, "set %x: selecting blk %x for replacement\n", set, regenerateBlkAddr(blk->tag, set));
	}
    return blk;
}

void
LRU::insertBlock(Addr addr, BlkType *blk, int master_id)
{
    if (!blk->isTouched) {
        tagsInUse++;
        blk->isTouched = true;
        if (!warmedUp && tagsInUse.value() >= warmupBound) {
            warmedUp = true;
            printf("The cache has completely warmed up --------------- \n");
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
	blk->set = extractSet(addr);
    // deal with what we are bringing in
    assert(master_id < cache->system->maxMasters());
    occupancies[master_id]++;
    blk->srcMasterId = master_id;

    unsigned set = extractSet(addr);
    //printf("in move to head 5 \n");
      //if(assoc!=1){
	sets[set].moveToHead(blk);//} // Change this line for Insertion policy
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
