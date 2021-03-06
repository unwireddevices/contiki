/*
 * Copyright (c) 2012, Thingsquare, http://www.thingsquare.com/.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * \addtogroup uip6
 * @{
 */

/**
 * \file
 *    Routing table manipulation
 */
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip.h"

#include "lib/list.h"
#include "lib/memb.h"
#include "net/nbr-table.h"

#include "../../../unwired/system-common.h"
#include "ota-main.h"
#include "dev/watchdog.h"
#include "system-common.h"

#include <string.h>

// #define GPRAM(NAME) __attribute__ ((section(CC_CONCAT(".gpram.",NAME))))
//CC_CONCAT(.gpram.,NAME)

////////////////////////////////////////////////////////////////////
#ifdef UNWDS_ROOT 
/* Define some settings */
#define ROUTELIST_SAVE_INTERVAL				(60 * 60 * CLOCK_SECOND) // Раз в час
#define MIN_NUM_ROUTE_SAVE					(20)

#define EEPROM_ADDR							(0x00060000)
#define MAGIC_BYTES_ADDR					(EEPROM_ADDR)
#define MAGIC_BYTES							(0xBABECAFE)
#define MAGIC_BYTES_LENGTH					(sizeof(uint32_t))
#define ROUTELIST_EEPROM_ADDR				(EEPROM_ADDR + MAGIC_BYTES_LENGTH)
#define ROUTELIST_LENGTH					(sizeof(uip_ds6_route_t) * UIP_CONF_MAX_ROUTES)

/* Save routelist to EEPROM process */
PROCESS(save_routelist_process, "Save routelist process");

route_table_eeprom_t *eeprom_flash = (route_table_eeprom_t*)EEPROM_ADDR;
#endif
////////////////////////////////////////////////////////////////////

/* A configurable function called after adding a new neighbor as next hop */
#ifdef NETSTACK_CONF_ROUTING_NEIGHBOR_ADDED_CALLBACK
void NETSTACK_CONF_ROUTING_NEIGHBOR_ADDED_CALLBACK(const linkaddr_t *addr);
#endif /* NETSTACK_CONF_ROUTING_NEIGHBOR_ADDED_CALLBACK */

/* A configurable function called after removing a next hop neighbor */
#ifdef NETSTACK_CONF_ROUTING_NEIGHBOR_REMOVED_CALLBACK
void NETSTACK_CONF_ROUTING_NEIGHBOR_REMOVED_CALLBACK(const linkaddr_t *addr);
#endif /* NETSTACK_CONF_ROUTING_NEIGHBOR_REMOVED_CALLBACK */

#if (UIP_CONF_MAX_ROUTES != 0)
/* The nbr_routes holds a neighbor table to be able to maintain
   information about what routes go through what neighbor. This
   neighbor table is registered with the central nbr-table repository
   so that it will be maintained along with the rest of the neighbor
   tables in the system. */
__attribute__ ((section(".gpram._nbr_routes_mem"))) static struct uip_ds6_route_neighbor_routes _nbr_routes_mem[NBR_TABLE_MAX_NEIGHBORS];

static nbr_table_t nbr_routes_struct = {0, 
										sizeof(struct uip_ds6_route_neighbor_routes), 
										NULL, 
										(nbr_table_item_t *)_nbr_routes_mem }; 

nbr_table_t *nbr_routes = &nbr_routes_struct;

__attribute__ ((section(".gpram.neighborroutememb_memb_count"))) static char neighborroutememb_memb_count[UIP_DS6_ROUTE_NB]; 
__attribute__ ((section(".gpram.neighborroutememb_memb_mem"))) static struct uip_ds6_route_neighbor_route neighborroutememb_memb_mem[UIP_DS6_ROUTE_NB]; 
static struct memb neighborroutememb = {sizeof(struct uip_ds6_route_neighbor_route), 
										UIP_DS6_ROUTE_NB, 
										neighborroutememb_memb_count, 
										(void *)neighborroutememb_memb_mem};

/* Each route is repressented by a uip_ds6_route_t structure and
   memory for each route is allocated from the routememb memory
   block. These routes are maintained on the routelist. */
static void *routelist_list = NULL;
static list_t routelist = (list_t)&routelist_list;

__attribute__ ((section(".gpram.routememb_memb_count"))) static char routememb_memb_count[UIP_DS6_ROUTE_NB]; 
static uip_ds6_route_t routememb_memb_mem[UIP_DS6_ROUTE_NB]; 
static struct memb routememb = {sizeof(uip_ds6_route_t), 
								UIP_DS6_ROUTE_NB, 
								routememb_memb_count, 
								(void *)routememb_memb_mem};

static int num_routes = 0;
static void rm_routelist_callback(nbr_table_item_t *ptr);

#endif /* (UIP_CONF_MAX_ROUTES != 0) */

/* Default routes are held on the defaultrouterlist and their
   structures are allocated from the defaultroutermemb memory block.*/
static void *defaultrouterlist_list = NULL;
static list_t defaultrouterlist = (list_t)&defaultrouterlist_list;

static char defaultroutermemb_memb_count[UIP_DS6_DEFRT_NB]; 
__attribute__ ((section(".gpram.defaultroutermemb_memb_mem"))) static uip_ds6_defrt_t defaultroutermemb_memb_mem[UIP_DS6_DEFRT_NB]; 
static struct memb defaultroutermemb = {sizeof(uip_ds6_defrt_t), 
										UIP_DS6_DEFRT_NB, 
										defaultroutermemb_memb_count, 
										(void *)defaultroutermemb_memb_mem};

#if UIP_DS6_NOTIFICATIONS
static void *notificationlist_list = NULL;
static list_t notificationlist = (list_t)&notificationlist_list;
#endif

#define DEBUG DEBUG_NONE
// #define DEBUG 1

#include "net/ip/uip-debug.h"

/*---------------------------------------------------------------------------*/
#if DEBUG != DEBUG_NONE
static void assert_nbr_routes_list_sane(void)
{
	uip_ds6_route_t *r;
	int count;

	/* Check if the route list has an infinite loop. */
	for(r = uip_ds6_route_head(),
		count = 0;
		r != NULL &&
		count < UIP_DS6_ROUTE_NB * 2;
		r = uip_ds6_route_next(r),
		count++);

	if(count > UIP_DS6_ROUTE_NB) 
	{
		printf("uip-ds6-route.c: assert_nbr_routes_list_sane route list is in infinite loop\n");
	}

	/* Make sure that the route list has as many entries as the
	num_routes vairable. */
	if(count < num_routes) 
	{
		printf("uip-ds6-route.c: assert_nbr_routes_list_sane too few entries on route list: should be %d, is %d, max %d\n",
			num_routes, count, UIP_CONF_MAX_ROUTES);
	}
}
#endif /* DEBUG != DEBUG_NONE */
/*---------------------------------------------------------------------------*/
#if UIP_DS6_NOTIFICATIONS
static void call_route_callback(int event, 
								uip_ipaddr_t *route,
								uip_ipaddr_t *nexthop)
{
	int num;
	struct uip_ds6_notification *n;
	for(n = list_head(notificationlist);
		n != NULL;
		n = list_item_next(n)) 
	{
		if(event == UIP_DS6_NOTIFICATION_DEFRT_ADD ||
		   event == UIP_DS6_NOTIFICATION_DEFRT_RM) 
		{
			num = list_length(defaultrouterlist);
		} 
		else 
		{
			num = num_routes;
		}

		n->callback(event, route, nexthop, num);
	}
}
/*---------------------------------------------------------------------------*/
void uip_ds6_notification_add(struct uip_ds6_notification *n,
	   						  uip_ds6_notification_callback c)
{
	if(n != NULL && c != NULL) 
	{
		n->callback = c;
		list_add(notificationlist, n);
	}
}
/*---------------------------------------------------------------------------*/
void uip_ds6_notification_rm(struct uip_ds6_notification *n)
{
	list_remove(notificationlist, n);
}
#endif
/*---------------------------------------------------------------------------*/
void uip_ds6_route_init(void)
{
////////////////////////////////////////////////////////////////////
#ifdef UNWDS_ROOT 
	if(spi_test())
	{
		if(load_routelist())
		{
			printf("Load routetable: ok\n");
			return;
		}
		else
		{
			printf("Load routetable: error\n");
		}
	}
	else
	{
		printf("Load routetable: could not access EEPROM\n");
	}

	/* Инициализируемся с нуля */
#if(UIP_CONF_MAX_ROUTES != 0)
	memb_init(&routememb);
	list_init(routelist);
	nbr_table_register(nbr_routes, (nbr_table_callback *)rm_routelist_callback);
#endif /* (UIP_CONF_MAX_ROUTES != 0) */

	memb_init(&defaultroutermemb);
	list_init(defaultrouterlist);

#if UIP_DS6_NOTIFICATIONS
	list_init(notificationlist);
#endif

#endif /* UNWDS_ROOT  */
////////////////////////////////////////////////////////////////////
}
#if (UIP_CONF_MAX_ROUTES != 0)
/*---------------------------------------------------------------------------*/
static uip_lladdr_t *uip_ds6_route_nexthop_lladdr(uip_ds6_route_t *route)
{
	if(route != NULL) 
	{
		return (uip_lladdr_t*)nbr_table_get_lladdr(nbr_routes, route->neighbor_routes);
	} 
	else 
	{
		return NULL;
	}
}
#endif /* (UIP_CONF_MAX_ROUTES != 0) */
/*---------------------------------------------------------------------------*/
uip_ipaddr_t *uip_ds6_route_nexthop(uip_ds6_route_t *route)
{
#if (UIP_CONF_MAX_ROUTES != 0)
	if(route != NULL) 
	{
		return uip_ds6_nbr_ipaddr_from_lladdr(uip_ds6_route_nexthop_lladdr(route));
	} 
	else 
	{
		return NULL;
	}
#else /* (UIP_CONF_MAX_ROUTES != 0) */
  	return NULL;
#endif /* (UIP_CONF_MAX_ROUTES != 0) */
}
/*---------------------------------------------------------------------------*/
uip_ds6_route_t *uip_ds6_route_head(void)
{
#if (UIP_CONF_MAX_ROUTES != 0)
  	return list_head(routelist);
#else /* (UIP_CONF_MAX_ROUTES != 0) */
  	return NULL;
#endif /* (UIP_CONF_MAX_ROUTES != 0) */
}
/*---------------------------------------------------------------------------*/
uip_ds6_route_t *uip_ds6_route_next(uip_ds6_route_t *r)
{
#if (UIP_CONF_MAX_ROUTES != 0)
	if(r != NULL) 
	{
		uip_ds6_route_t *n = list_item_next(r);
		return n;
	}
#endif /* (UIP_CONF_MAX_ROUTES != 0) */
  	return NULL;
}
/*---------------------------------------------------------------------------*/
int uip_ds6_route_is_nexthop(const uip_ipaddr_t *ipaddr)
{
#if (UIP_CONF_MAX_ROUTES != 0)
	const uip_lladdr_t *lladdr;
	lladdr = uip_ds6_nbr_lladdr_from_ipaddr(ipaddr);

	if(lladdr == NULL) 
	{
		return 0;
	}

  	return nbr_table_get_from_lladdr(nbr_routes, (linkaddr_t *)lladdr) != NULL;
#else /* (UIP_CONF_MAX_ROUTES != 0) */
  	return 0;
#endif /* (UIP_CONF_MAX_ROUTES != 0) */
}
/*---------------------------------------------------------------------------*/
int uip_ds6_route_num_routes(void)
{
#if (UIP_CONF_MAX_ROUTES != 0)
  	return num_routes;
#else /* (UIP_CONF_MAX_ROUTES != 0) */
  	return 0;
#endif /* (UIP_CONF_MAX_ROUTES != 0) */
}
/*---------------------------------------------------------------------------*/
uip_ds6_route_t *uip_ds6_route_lookup(uip_ipaddr_t *addr)
{
#if (UIP_CONF_MAX_ROUTES != 0)
	uip_ds6_route_t *r;
	uip_ds6_route_t *found_route;
	uint8_t longestmatch;

	PRINTF("uip-ds6-route: Looking up route for ");
	PRINT6ADDR(addr);
	PRINTF("\n");

	found_route = NULL;
	longestmatch = 0;
	
	for(r = uip_ds6_route_head();
		r != NULL;
		r = uip_ds6_route_next(r)) 
	{
		uip_ipaddr_t ipaddr_decompress;
		decompress_uip_ipaddr_t(&ipaddr_decompress, &r->ipaddr);
		if( r->length >= longestmatch &&
			uip_ipaddr_prefixcmp(addr, &ipaddr_decompress, r->length)) 
		{
			longestmatch = r->length;
			found_route = r;
			
			/* check if total match - e.g. all 128 bits do match */
			if(longestmatch == 128) 
			{
				break;
			}
		}
	}

	if(found_route != NULL) 
	{
		PRINTF("uip-ds6-route: Found route: ");
		PRINT6ADDR(addr);
		PRINTF(" via ");
		PRINT6ADDR(uip_ds6_route_nexthop(found_route));
		PRINTF("\n");
	} 
	else 
	{
		PRINTF("uip-ds6-route: No route found\n");
	}

	if(found_route != NULL && found_route != list_head(routelist)) 
	{
		/* If we found a route, we put it at the start of the routeslist
		list. The list is ordered by how recently we looked them up:
		the least recently used route will be at the end of the
		list - for fast lookups (assuming multiple packets to the same node). */

		list_remove(routelist, found_route);
		list_push(routelist, found_route);
	}

	return found_route;
#else /* (UIP_CONF_MAX_ROUTES != 0) */
  	return NULL;
#endif /* (UIP_CONF_MAX_ROUTES != 0) */
}
/*---------------------------------------------------------------------------*/
uip_ds6_route_t *uip_ds6_route_add(uip_ipaddr_t *ipaddr, 
		  						   uint8_t length,
		  						   uip_ipaddr_t *nexthop)
{
#if (UIP_CONF_MAX_ROUTES != 0)
	uip_ds6_route_t *r;
	struct uip_ds6_route_neighbor_route *nbrr;

#if DEBUG != DEBUG_NONE
  	assert_nbr_routes_list_sane();
#endif /* DEBUG != DEBUG_NONE */

	/* Get link-layer address of next hop, make sure it is in neighbor table */
	const uip_lladdr_t *nexthop_lladdr = uip_ds6_nbr_lladdr_from_ipaddr(nexthop);
	if(nexthop_lladdr == NULL) 
	{
		PRINTF("uip_ds6_route_add: neighbor link-local address unknown for ");
		PRINT6ADDR(nexthop);
		PRINTF("\n");
		return NULL;
	}

	/* First make sure that we don't add a route twice. If we find an
	existing route for our destination, we'll delete the old
	one first. */
	r = uip_ds6_route_lookup(ipaddr);
	if(r != NULL) 
	{
		uip_ipaddr_t *current_nexthop;
		current_nexthop = uip_ds6_route_nexthop(r);
		if(current_nexthop != NULL && uip_ipaddr_cmp(nexthop, current_nexthop)) 
		{
			/* no need to update route - already correct! */
			return r;
		}
		PRINTF("uip_ds6_route_add: old route for ");
		PRINT6ADDR(ipaddr);
		PRINTF(" found, deleting it\n");

		uip_ds6_route_rm(r);
	}
	{
		struct uip_ds6_route_neighbor_routes *routes;
		/* If there is no routing entry, create one. We first need to
		check if we have room for this route. If not, we remove the
		least recently used one we have. */

		if(uip_ds6_route_num_routes() == UIP_DS6_ROUTE_NB) 
		{
			uip_ds6_route_t *oldest;
			oldest = NULL;
#if UIP_DS6_ROUTE_REMOVE_LEAST_RECENTLY_USED
			/* Removing the oldest route entry from the route table. The
			least recently used route is the first route on the list. */
			oldest = list_tail(routelist);
#endif
			if(oldest == NULL) 
			{
				return NULL;
			}
			uip_ipaddr_t ipaddr_decompress;
			decompress_uip_ipaddr_t(&ipaddr_decompress, &oldest->ipaddr);
			//PRINTF("***DECOMPRESS ADD***\n");
			PRINTF("uip_ds6_route_add: dropping route to ");
			PRINT6ADDR(&ipaddr_decompress);
			PRINTF("\n");
			uip_ds6_route_rm(oldest);
		}

		/* Every neighbor on our neighbor table holds a struct
		uip_ds6_route_neighbor_routes which holds a list of routes that
		go through the neighbor. We add our route entry to this list.

		We first check to see if we already have this neighbor in our
		nbr_route table. If so, the neighbor already has a route entry
		list.
		*/
		routes = nbr_table_get_from_lladdr ( nbr_routes, (linkaddr_t *)nexthop_lladdr);

		if(routes == NULL) 
		{
			/* If the neighbor did not have an entry in our neighbor table,
			we create one. The nbr_table_add_lladdr() function returns a
			pointer to a pointer that we may use for our own purposes. We
			initialize this pointer with the list of routing entries that
			are attached to this neighbor. */
			routes = nbr_table_add_lladdr(nbr_routes,
									     (linkaddr_t *)nexthop_lladdr,
										 NBR_TABLE_REASON_ROUTE, NULL);
			if(routes == NULL) 
			{
				/* This should not happen, as we explicitly deallocated one
				route table entry above. */
				PRINTF("uip_ds6_route_add: could not allocate neighbor table entry\n");
				return NULL;
			}
			LIST_STRUCT_INIT(routes, route_list);
#ifdef NETSTACK_CONF_ROUTING_NEIGHBOR_ADDED_CALLBACK
	  		NETSTACK_CONF_ROUTING_NEIGHBOR_ADDED_CALLBACK((const linkaddr_t *)nexthop_lladdr);
#endif
		}

		/* Allocate a routing entry and populate it. */
		r = memb_alloc(&routememb);

		if(r == NULL) 
		{
			/* This should not happen, as we explicitly deallocated one
			route table entry above. */
			PRINTF("uip_ds6_route_add: could not allocate route\n");
			return NULL;
		}

		/* add new routes first - assuming that there is a reason to add this
		and that there is a packet coming soon. */
		list_push(routelist, r);

		nbrr = memb_alloc(&neighborroutememb);
		if(nbrr == NULL) 
		{
			/* This should not happen, as we explicitly deallocated one
			route table entry above. */
			PRINTF("uip_ds6_route_add: could not allocate neighbor route list entry\n");
			memb_free(&routememb, r);
			return NULL;
		}

		nbrr->route = r;

		/* Add the route to this neighbor */
		list_add(routes->route_list, nbrr);
		r->neighbor_routes = routes;
		num_routes++;

		PRINTF("uip_ds6_route_add num %d\n", num_routes);

		/* lock this entry so that nexthop is not removed */
		nbr_table_lock(nbr_routes, routes);
	}	
  
	//uip_ipaddr_copy(&(r->ipaddr), ipaddr);
	//PRINTF("***COMPRESS ADD***\n");
	//PRINT6ADDR(ipaddr);
	compress_uip_ipaddr_t(ipaddr, &(r->ipaddr)); 
	
	r->length = length;
////////////////////////////////////////////////////////////////////
#ifdef UNWDS_ROOT 
  	r->counter = 0xFFFF;
#endif
////////////////////////////////////////////////////////////////////

#ifdef UIP_DS6_ROUTE_STATE_TYPE
  	memset(&r->state, 0, sizeof(UIP_DS6_ROUTE_STATE_TYPE));
#endif

	PRINTF("uip_ds6_route_add: adding route: ");
	PRINT6ADDR(ipaddr);
	PRINTF(" via ");
	PRINT6ADDR(nexthop);
	PRINTF("\n");
	ANNOTATE("#L %u 1;blue\n", nexthop->u8[sizeof(uip_ipaddr_t) - 1]);

#if UIP_DS6_NOTIFICATIONS
  	call_route_callback(UIP_DS6_NOTIFICATION_ROUTE_ADD, ipaddr, nexthop);
#endif

#if DEBUG != DEBUG_NONE
  	assert_nbr_routes_list_sane();
#endif /* DEBUG != DEBUG_NONE */
  	return r;

#else /* (UIP_CONF_MAX_ROUTES != 0) */
  	return NULL;
#endif /* (UIP_CONF_MAX_ROUTES != 0) */
}

/*---------------------------------------------------------------------------*/
void uip_ds6_route_rm(uip_ds6_route_t *route)
{
#if (UIP_CONF_MAX_ROUTES != 0)
  	struct uip_ds6_route_neighbor_route *neighbor_route;
#if DEBUG != DEBUG_NONE
  	assert_nbr_routes_list_sane();
#endif /* DEBUG != DEBUG_NONE */
	if(route != NULL && route->neighbor_routes != NULL) 
	{
		uip_ipaddr_t ipaddr_decompress;
		decompress_uip_ipaddr_t(&ipaddr_decompress, &route->ipaddr);
		//PRINTF("***DECOMPRESS RM***\n");

		PRINTF("uip_ds6_route_rm: removing route: ");
		PRINT6ADDR(&ipaddr_decompress);
		PRINTF("\n");

		/* Remove the route from the route list */
		list_remove(routelist, route);

		/* Find the corresponding neighbor_route and remove it. */
		for(neighbor_route = list_head(route->neighbor_routes->route_list);
			neighbor_route != NULL && neighbor_route->route != route;
			neighbor_route = list_item_next(neighbor_route));

		if(neighbor_route == NULL) 
		{
			PRINTF("uip_ds6_route_rm: neighbor_route was NULL for ");
			//decompress_uip_ipaddr_t(&ipaddr_decompress, &route->ipaddr);
			uip_debug_ipaddr_print(&ipaddr_decompress);
			PRINTF("\n");
		}
		list_remove(route->neighbor_routes->route_list, neighbor_route);
		if(list_head(route->neighbor_routes->route_list) == NULL) 
		{
			/* If this was the only route using this neighbor, remove the
			neighbor from the table - this implicitly unlocks nexthop */
#if (DEBUG) & DEBUG_ANNOTATE
			uip_ipaddr_t *nexthop = uip_ds6_route_nexthop(route);
			if(nexthop != NULL) 
			{
				ANNOTATE("#L %u 0\n", nexthop->u8[sizeof(uip_ipaddr_t) - 1]);
			}	
#endif /* (DEBUG) & DEBUG_ANNOTATE */
			PRINTF("uip_ds6_route_rm: removing neighbor too\n");
			nbr_table_remove(nbr_routes, route->neighbor_routes->route_list);
#ifdef NETSTACK_CONF_ROUTING_NEIGHBOR_REMOVED_CALLBACK
	  		NETSTACK_CONF_ROUTING_NEIGHBOR_REMOVED_CALLBACK(
				(const linkaddr_t *)nbr_table_get_lladdr(nbr_routes, route->neighbor_routes->route_list));
#endif
		}
		memb_free(&routememb, route);
		memb_free(&neighborroutememb, neighbor_route);

		num_routes--;

		PRINTF("uip_ds6_route_rm num %d\n", num_routes);

#if UIP_DS6_NOTIFICATIONS
		call_route_callback(UIP_DS6_NOTIFICATION_ROUTE_RM,
							&ipaddr_decompress, 
							uip_ds6_route_nexthop(route));
#endif
  	}

#if DEBUG != DEBUG_NONE
  	assert_nbr_routes_list_sane();
#endif /* DEBUG != DEBUG_NONE */

#endif /* (UIP_CONF_MAX_ROUTES != 0) */
  	return;
}
#if (UIP_CONF_MAX_ROUTES != 0)
/*---------------------------------------------------------------------------*/
static void rm_routelist(struct uip_ds6_route_neighbor_routes *routes)
{
#if DEBUG != DEBUG_NONE
  	assert_nbr_routes_list_sane();
#endif /* DEBUG != DEBUG_NONE */
  	PRINTF("uip_ds6_route_rm_routelist\n");
	if(routes != NULL && routes->route_list != NULL) 
	{
		struct uip_ds6_route_neighbor_route *r;
		r = list_head(routes->route_list);
		while(r != NULL) 
		{
			uip_ds6_route_rm(r->route);
			r = list_head(routes->route_list);
		}
		nbr_table_remove(nbr_routes, routes);
	}
#if DEBUG != DEBUG_NONE
  	assert_nbr_routes_list_sane();
#endif /* DEBUG != DEBUG_NONE */
}
/*---------------------------------------------------------------------------*/
static void rm_routelist_callback(nbr_table_item_t *ptr)
{
	rm_routelist((struct uip_ds6_route_neighbor_routes *)ptr);
}
#endif /* (UIP_CONF_MAX_ROUTES != 0) */
/*---------------------------------------------------------------------------*/
void uip_ds6_route_rm_by_nexthop(uip_ipaddr_t *nexthop)
{
#if (UIP_CONF_MAX_ROUTES != 0)
	/* Get routing entry list of this neighbor */
	const uip_lladdr_t *nexthop_lladdr;
	struct uip_ds6_route_neighbor_routes *routes;

	nexthop_lladdr = uip_ds6_nbr_lladdr_from_ipaddr(nexthop);
	routes = nbr_table_get_from_lladdr(nbr_routes, (linkaddr_t *)nexthop_lladdr);
	rm_routelist(routes);
#endif /* (UIP_CONF_MAX_ROUTES != 0) */
}
/*---------------------------------------------------------------------------*/
uip_ds6_defrt_t *
uip_ds6_defrt_add(uip_ipaddr_t *ipaddr, unsigned long interval)
{
  	uip_ds6_defrt_t *d;

#if DEBUG != DEBUG_NONE
  	assert_nbr_routes_list_sane();
#endif /* DEBUG != DEBUG_NONE */

	PRINTF("uip_ds6_defrt_add\n");
	d = uip_ds6_defrt_lookup(ipaddr);
	if(d == NULL) 
	{
		d = memb_alloc(&defaultroutermemb);
		if(d == NULL) 
		{
			PRINTF("uip_ds6_defrt_add: could not add default route to ");
			PRINT6ADDR(ipaddr);
			PRINTF(", out of memory\n");
			return NULL;
		} 
		else 
		{
			PRINTF("uip_ds6_defrt_add: adding default route to ");
			PRINT6ADDR(ipaddr);
			PRINTF("\n");
		}

		list_push(defaultrouterlist, d);
	}

	uip_ipaddr_copy(&d->ipaddr, ipaddr);
	if(interval != 0) 
	{
		stimer_set(&d->lifetime, interval);
		d->isinfinite = 0;
	} 
	else 
	{
		d->isinfinite = 1;
	}

	ANNOTATE("#L %u 1\n", ipaddr->u8[sizeof(uip_ipaddr_t) - 1]);

#if UIP_DS6_NOTIFICATIONS
  	call_route_callback(UIP_DS6_NOTIFICATION_DEFRT_ADD, ipaddr, ipaddr);
#endif

#if DEBUG != DEBUG_NONE
  	assert_nbr_routes_list_sane();
#endif /* DEBUG != DEBUG_NONE */

  	return d;
}
/*---------------------------------------------------------------------------*/
void uip_ds6_defrt_rm(uip_ds6_defrt_t *defrt)
{
  	uip_ds6_defrt_t *d;

#if DEBUG != DEBUG_NONE
	assert_nbr_routes_list_sane();
#endif /* DEBUG != DEBUG_NONE */

	/* Make sure that the defrt is in the list before we remove it. */
	for(d = list_head(defaultrouterlist);
		d != NULL;
		d = list_item_next(d)) 
	{
		if(d == defrt) 
		{
			PRINTF("Removing default route\n");
			list_remove(defaultrouterlist, defrt);
			memb_free(&defaultroutermemb, defrt);
			ANNOTATE("#L %u 0\n", defrt->ipaddr.u8[sizeof(uip_ipaddr_t) - 1]);
#if UIP_DS6_NOTIFICATIONS
			call_route_callback(UIP_DS6_NOTIFICATION_DEFRT_RM, &defrt->ipaddr, &defrt->ipaddr);
#endif
			return;
		}
	}
#if DEBUG != DEBUG_NONE
  	assert_nbr_routes_list_sane();
#endif /* DEBUG != DEBUG_NONE */

}
/*---------------------------------------------------------------------------*/
uip_ds6_defrt_t *uip_ds6_defrt_lookup(uip_ipaddr_t *ipaddr)
{
	uip_ds6_defrt_t *d;
	for(d = list_head(defaultrouterlist);
		d != NULL;
		d = list_item_next(d)) 
	{
		if(uip_ipaddr_cmp(&d->ipaddr, ipaddr)) 
		{
			return d;
		}
	}
	return NULL;
}
/*---------------------------------------------------------------------------*/
uip_ipaddr_t *uip_ds6_defrt_choose(void)
{
	uip_ds6_defrt_t *d;
	uip_ds6_nbr_t *bestnbr;
	uip_ipaddr_t *addr;

	addr = NULL;
	for(d = list_head(defaultrouterlist);
		d != NULL;
		d = list_item_next(d)) 
	{
		PRINTF("Defrt, IP address ");
		PRINT6ADDR(&d->ipaddr);
		PRINTF("\n");
		bestnbr = uip_ds6_nbr_lookup(&d->ipaddr);
		if(bestnbr != NULL && bestnbr->state != NBR_INCOMPLETE) 
		{
			PRINTF("Defrt found, IP address ");
			PRINT6ADDR(&d->ipaddr);
			PRINTF("\n");
			return &d->ipaddr;
		} 
		else 
		{
			addr = &d->ipaddr;
			PRINTF("Defrt INCOMPLETE found, IP address ");
			PRINT6ADDR(&d->ipaddr);
			PRINTF("\n");
		}
	}
	return addr;
}
/*---------------------------------------------------------------------------*/
void uip_ds6_defrt_periodic(void)
{
	uip_ds6_defrt_t *d;
	d = list_head(defaultrouterlist);
	while(d != NULL) 
	{
		if(!d->isinfinite &&
		stimer_expired(&d->lifetime)) 
		{
			PRINTF("uip_ds6_defrt_periodic: defrt lifetime expired\n");
			uip_ds6_defrt_rm(d);
			d = list_head(defaultrouterlist);
		} 
		else 
		{
			d = list_item_next(d);
		}
	}
}
/*---------------------------------------------------------------------------*/
void compress_uip_ipaddr_t(uip_ipaddr_t *addr_in, uip_ipaddr_compressed_t *addr_out) 
{
	addr_out->u16[0] = addr_in->u16[4];
	addr_out->u16[1] = addr_in->u16[5];
	addr_out->u16[2] = addr_in->u16[6];
	addr_out->u16[3] = addr_in->u16[7];
}
/*---------------------------------------------------------------------------*/
void decompress_uip_ipaddr_t(uip_ipaddr_t *addr_out, uip_ipaddr_compressed_t *addr_in)
{
	addr_out->u8[0] = 0xFD;
	addr_out->u8[1] = 0;
	addr_out->u16[1] = 0;
	addr_out->u16[2] = 0;
	addr_out->u16[3] = 0;
	addr_out->u16[4] = addr_in->u16[0];
	addr_out->u16[5] = addr_in->u16[1];
	addr_out->u16[6] = addr_in->u16[2];
	addr_out->u16[7] = addr_in->u16[3];
}
/*---------------------------------------------------------------------------*/
#ifdef UNWDS_ROOT 
// uip_ds6_route_t *
// uip_ds6_route_serial_lookup(uint32_t serial)
// {
  // uip_ds6_route_t *r;
  // uip_ds6_route_t *found_route;

  // PRINTF("uip-ds6-route: Looking up route for serial: %lu\n", serial);

  // found_route = NULL;
  
  // for(r = uip_ds6_route_head();
	// r != NULL;
	// r = uip_ds6_route_next(r)) 
  // {			
	// if(serial == r->serial)
	// {
	  // found_route = r;
	  // break;
	// }
  // }

  // if(found_route != NULL) 
  // {
	// PRINTF("uip-ds6-route: Found route for serial: %lu \n", serial);
	// PRINTF("uip-ds6-route: Addr: ");
	// uip_ipaddr_t ipaddr_decompress;
	// decompress_uip_ipaddr_t(&ipaddr_decompress, &(r->ipaddr));
	// PRINT6ADDR(&ipaddr_decompress);
	// PRINTF("\n");
  // } 
  // else 
  // {
	// PRINTF("uip-ds6-route: No route found\n");
  // }

  // if(found_route != NULL && found_route != list_head(routelist)) 
  // {
	// /* If we found a route, we put it at the start of the routeslist
	// list. The list is ordered by how recently we looked them up:
	// the least recently used route will be at the end of the
	// list - for fast lookups (assuming multiple packets to the same node). */

	// list_remove(routelist, found_route);
	// list_push(routelist, found_route);
  // }

  // return found_route;
// }
/*---------------------------------------------------------------------------*/
void add_route(uip_ip6addr_t *addr, uint16_t nonce) 
{
	uip_ds6_route_t *r = uip_ds6_route_lookup(addr);
	
	if(r != NULL)
	{
		r->nonce = nonce;
		r->counter = 0xFFFF;
	}
}
/*---------------------------------------------------------------------------*/
// uip_ip6addr_t find_addr(uint32_t serial)
// {
  // PRINTF("uip-ds6-route: Find addr from serial: %lu\n", serial);
  // uip_ds6_route_t *r = uip_ds6_route_serial_lookup(serial);
  // uip_ip6addr_t addr;
  
  // if(r != NULL)
  // {
	// decompress_uip_ipaddr_t(&addr, &(r->ipaddr));
	// PRINTF("uip-ds6-route: Addr: ");
	// PRINT6ADDR(&addr);
	// PRINTF("\n");
  // }
  // else
  // {
	// uip_ip6addr(&addr, 0, 0, 0, 0, 0, 0, 0, 0); //Адрес не найден
	// PRINTF("uip-ds6-route: No addr found\n");
	// PRINTF("Addr: ");
  // }
  
  // return addr;
// }
/*---------------------------------------------------------------------------*/
uint16_t get_nonce(uip_ip6addr_t *addr)
{
	uip_ds6_route_t *r = uip_ds6_route_lookup(addr);
	
	if(r != NULL)
		return r->nonce;
	
	return 0;
}
/*---------------------------------------------------------------------------*/
void unlock_addr(uip_ip6addr_t *addr)
{
	uip_ds6_route_t *r = uip_ds6_route_lookup(addr);
	
	if(r != NULL)
	{
		if(r->counter == 0xFFFF) //Разблокируем счетчик
		r->counter = 0;	
	}		
}
/*---------------------------------------------------------------------------*/
bool valid_counter(uip_ip6addr_t *addr, uint16_t counter)
{
	uip_ds6_route_t *r = uip_ds6_route_lookup(addr);
	
	if(r != NULL)
	{
		if(r->counter < counter)//Проверка на активность. 
		{
			r->counter = counter;
			return true;
		}
	}
	return false;
}

/*---------------------------------------------------------------------------*/
bool load_routelist(void)
{
	/* Проверяем установлена ли флешка */
	bool eeprom_access = ext_flash_open();
	if(eeprom_access)
	{
		/* Проверяет есть ли сохранённая таблица маршрутизации, если есть, то загружает её */
		uint32_t magic_bytes; 

		/* Считываем магические байты */
		ext_flash_read((uint32_t)&eeprom_flash->magic_bytes, sizeof(eeprom_flash->magic_bytes), (uint8_t*)&magic_bytes);

		if(magic_bytes != MAGIC_BYTES)
		{
			PRINTF("load_routelist: flash is empty\n");
			
			/* Start save routelist to EEPROM process */
			process_start(&save_routelist_process, NULL);

			ext_flash_close();
			return false;
		}

		ext_flash_read((uint32_t)&eeprom_flash->defaultroutermemb, sizeof(eeprom_flash->defaultroutermemb), (uint8_t*)&defaultroutermemb);
		ext_flash_read((uint32_t)&eeprom_flash->nbr_routes, sizeof(eeprom_flash->nbr_routes), (uint8_t*)&nbr_routes);
		ext_flash_read((uint32_t)&eeprom_flash->nbr_routes_struct, sizeof(eeprom_flash->nbr_routes_struct), (uint8_t*)&nbr_routes_struct);
		ext_flash_read((uint32_t)&eeprom_flash->neighborroutememb, sizeof(eeprom_flash->neighborroutememb), (uint8_t*)&neighborroutememb);
		ext_flash_read((uint32_t)&eeprom_flash->routememb, sizeof(eeprom_flash->routememb), (uint8_t*)&routememb);
		ext_flash_read((uint32_t)&eeprom_flash->defaultrouterlist_list, sizeof(eeprom_flash->defaultrouterlist_list), (uint8_t*)&defaultrouterlist_list);
		ext_flash_read((uint32_t)&eeprom_flash->defaultroutermemb_memb_count, sizeof(eeprom_flash->defaultroutermemb_memb_count), (uint8_t*)&defaultroutermemb_memb_count[0]);
		ext_flash_read((uint32_t)&eeprom_flash->notificationlist_list, sizeof(eeprom_flash->notificationlist_list), (uint8_t*)&notificationlist_list);
		ext_flash_read((uint32_t)&eeprom_flash->num_routes, sizeof(eeprom_flash->num_routes), (uint8_t*)&num_routes);
		ext_flash_read((uint32_t)&eeprom_flash->routelist_list, sizeof(eeprom_flash->routelist_list), (uint8_t*)&routelist_list);
		ext_flash_read((uint32_t)&eeprom_flash->routememb_memb_mem, sizeof(eeprom_flash->routememb_memb_mem), (uint8_t*)&routememb_memb_mem[0]);
		ext_flash_read((uint32_t)&eeprom_flash->_nbr_routes_mem, sizeof(eeprom_flash->_nbr_routes_mem), (uint8_t*)&_nbr_routes_mem[0]);
		ext_flash_read((uint32_t)&eeprom_flash->defaultroutermemb_memb_mem, sizeof(eeprom_flash->defaultroutermemb_memb_mem), (uint8_t*)&defaultroutermemb_memb_mem[0]);
		ext_flash_read((uint32_t)&eeprom_flash->neighborroutememb_memb_count, sizeof(eeprom_flash->neighborroutememb_memb_count), (uint8_t*)&neighborroutememb_memb_count[0]);
		ext_flash_read((uint32_t)&eeprom_flash->neighborroutememb_memb_mem, sizeof(eeprom_flash->neighborroutememb_memb_mem), (uint8_t*)&neighborroutememb_memb_mem[0]);
		ext_flash_read((uint32_t)&eeprom_flash->routememb_memb_count, sizeof(eeprom_flash->routememb_memb_count), (uint8_t*)&routememb_memb_count[0]);

		/* CRC16 */
		uint16_t crc16, crc16_eeprom;

		crc16 = crc16_add((uint8_t*)&defaultroutermemb, sizeof(eeprom_flash->defaultroutermemb), 0x0000);
		crc16 = crc16_add((uint8_t*)&nbr_routes, sizeof(eeprom_flash->nbr_routes), crc16);
		crc16 = crc16_add((uint8_t*)&nbr_routes_struct, sizeof(eeprom_flash->nbr_routes_struct), crc16);
		crc16 = crc16_add((uint8_t*)&neighborroutememb, sizeof(eeprom_flash->neighborroutememb), crc16);
		crc16 = crc16_add((uint8_t*)&routememb, sizeof(eeprom_flash->routememb), crc16);
		crc16 = crc16_add((uint8_t*)&defaultrouterlist_list, sizeof(eeprom_flash->defaultrouterlist_list), crc16);
		crc16 = crc16_add((uint8_t*)&defaultroutermemb_memb_count[0], sizeof(eeprom_flash->defaultroutermemb_memb_count), crc16);
		crc16 = crc16_add((uint8_t*)&notificationlist_list, sizeof(eeprom_flash->notificationlist_list), crc16);
		crc16 = crc16_add((uint8_t*)&num_routes, sizeof(eeprom_flash->num_routes), crc16);
		crc16 = crc16_add((uint8_t*)&routelist_list, sizeof(eeprom_flash->routelist_list), crc16);
		crc16 = crc16_add((uint8_t*)&routememb_memb_mem[0], sizeof(eeprom_flash->routememb_memb_mem), crc16);
		crc16 = crc16_add((uint8_t*)&_nbr_routes_mem[0], sizeof(eeprom_flash->_nbr_routes_mem), crc16);
		crc16 = crc16_add((uint8_t*)&defaultroutermemb_memb_mem[0], sizeof(eeprom_flash->defaultroutermemb_memb_mem), crc16);
		crc16 = crc16_add((uint8_t*)&neighborroutememb_memb_count[0], sizeof(eeprom_flash->neighborroutememb_memb_count), crc16);
		crc16 = crc16_add((uint8_t*)&neighborroutememb_memb_mem[0], sizeof(eeprom_flash->neighborroutememb_memb_mem), crc16);
		crc16 = crc16_add((uint8_t*)&routememb_memb_count[0], sizeof(eeprom_flash->routememb_memb_count), crc16);

		ext_flash_read((uint32_t)&eeprom_flash->crc16, sizeof(eeprom_flash->crc16), (uint8_t*)&crc16_eeprom);
		PRINTF("crc16_route_table: 0x%04x == 0x%04x\n", crc16, crc16_eeprom);

		if(crc16 != crc16_eeprom)
		{
			/* Контрольная сумма не верна */
			/* Очищаем странницу с таблицей маршрутизации */
			ext_flash_erase((uint32_t)eeprom_flash, sizeof(*eeprom_flash));
			ext_flash_close();

			/* Start save routelist to EEPROM process */
			process_start(&save_routelist_process, NULL);
			
			return false;
		}

		/* Start save routelist to EEPROM process */
		process_start(&save_routelist_process, NULL);
		ext_flash_close();
		return true;
	}
	else
	{
		PRINTF("load_routelist: Could not access EEPROM\n");
		ext_flash_close();
		return false;
	}
}

/*---------------------------------------------------------------------------*/
bool save_routelist(void)
{
	/* Проверяем установлена ли флешка */
	bool eeprom_access = ext_flash_open();
	if(eeprom_access)
	{
		PRINTF("eeprom_flash addr: 0x%08lx len: 0x%08x\n", (uint32_t)eeprom_flash, sizeof(*eeprom_flash));
		PRINTF("magic_bytes addr: 0x%08lx len: 0x%08x\n", (uint32_t)&eeprom_flash->magic_bytes, sizeof(eeprom_flash->magic_bytes));
		PRINTF("crc16 addr: 0x%08lx len: 0x%08x\n", (uint32_t)&eeprom_flash->crc16, sizeof(eeprom_flash->crc16));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (defaultroutermemb)\n", (uint32_t)&defaultroutermemb, (uint32_t)&eeprom_flash->defaultroutermemb, sizeof(eeprom_flash->defaultroutermemb));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (nbr_routes)\n", (uint32_t)&nbr_routes, (uint32_t)&eeprom_flash->nbr_routes, sizeof(eeprom_flash->nbr_routes));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (nbr_routes_struct)\n", (uint32_t)&nbr_routes_struct, (uint32_t)&eeprom_flash->nbr_routes_struct, sizeof(eeprom_flash->nbr_routes_struct));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (neighborroutememb)\n", (uint32_t)&neighborroutememb, (uint32_t)&eeprom_flash->neighborroutememb, sizeof(eeprom_flash->neighborroutememb));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (routememb)\n", (uint32_t)&routememb, (uint32_t)&eeprom_flash->routememb, sizeof(eeprom_flash->routememb));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (defaultrouterlist_list)\n", (uint32_t)&defaultrouterlist_list, (uint32_t)&eeprom_flash->defaultrouterlist_list, sizeof(eeprom_flash->defaultrouterlist_list));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (defaultroutermemb_memb_count)\n", (uint32_t)&defaultroutermemb_memb_count[0], (uint32_t)&eeprom_flash->defaultroutermemb_memb_count[0], sizeof(eeprom_flash->defaultroutermemb_memb_count));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (notificationlist_list)\n", (uint32_t)&notificationlist_list, (uint32_t)&eeprom_flash->notificationlist_list, sizeof(eeprom_flash->notificationlist_list));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (num_routes)\n", (uint32_t)&num_routes, (uint32_t)&eeprom_flash->num_routes, sizeof(eeprom_flash->num_routes));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (routelist_list)\n", (uint32_t)&routelist_list, (uint32_t)&eeprom_flash->routelist_list, sizeof(eeprom_flash->routelist_list));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (routememb_memb_mem)\n", (uint32_t)&routememb_memb_mem[0], (uint32_t)&eeprom_flash->routememb_memb_mem, sizeof(eeprom_flash->routememb_memb_mem));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (_nbr_routes_mem)\n", (uint32_t)&_nbr_routes_mem[0], (uint32_t)&eeprom_flash->_nbr_routes_mem, sizeof(eeprom_flash->_nbr_routes_mem));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (defaultroutermemb_memb_mem)\n", (uint32_t)&defaultroutermemb_memb_mem[0], (uint32_t)&eeprom_flash->defaultroutermemb_memb_mem, sizeof(eeprom_flash->defaultroutermemb_memb_mem));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (neighborroutememb_memb_count)\n", (uint32_t)&neighborroutememb_memb_count[0], (uint32_t)&eeprom_flash->neighborroutememb_memb_count, sizeof(eeprom_flash->neighborroutememb_memb_count));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (neighborroutememb_memb_mem)\n", (uint32_t)&neighborroutememb_memb_mem[0], (uint32_t)&eeprom_flash->neighborroutememb_memb_mem, sizeof(eeprom_flash->neighborroutememb_memb_mem));
		PRINTF("Copy from 0x%08lx to 0x%08lx len: 0x%08x (routememb_memb_count)\n", (uint32_t)&routememb_memb_count[0], (uint32_t)&eeprom_flash->routememb_memb_count, sizeof(eeprom_flash->routememb_memb_count));

		/* ERASE FLASH */
		ext_flash_erase((uint32_t)eeprom_flash, sizeof(*eeprom_flash));
		watchdog_periodic();

		/* SAVE ROUTE TABLE */
		ext_flash_write((uint32_t)&eeprom_flash->defaultroutermemb, sizeof(eeprom_flash->defaultroutermemb), (uint8_t*)&defaultroutermemb);
		ext_flash_write((uint32_t)&eeprom_flash->nbr_routes, sizeof(eeprom_flash->nbr_routes), (uint8_t*)&nbr_routes);
		ext_flash_write((uint32_t)&eeprom_flash->nbr_routes_struct, sizeof(eeprom_flash->nbr_routes_struct), (uint8_t*)&nbr_routes_struct);
		ext_flash_write((uint32_t)&eeprom_flash->neighborroutememb, sizeof(eeprom_flash->neighborroutememb), (uint8_t*)&neighborroutememb);
		ext_flash_write((uint32_t)&eeprom_flash->routememb, sizeof(eeprom_flash->routememb), (uint8_t*)&routememb);
		ext_flash_write((uint32_t)&eeprom_flash->defaultrouterlist_list, sizeof(eeprom_flash->defaultrouterlist_list), (uint8_t*)&defaultrouterlist_list);
		ext_flash_write((uint32_t)&eeprom_flash->defaultroutermemb_memb_count, sizeof(eeprom_flash->defaultroutermemb_memb_count), (uint8_t*)&defaultroutermemb_memb_count[0]);
		ext_flash_write((uint32_t)&eeprom_flash->notificationlist_list, sizeof(eeprom_flash->notificationlist_list), (uint8_t*)&notificationlist_list);
		ext_flash_write((uint32_t)&eeprom_flash->num_routes, sizeof(eeprom_flash->num_routes), (uint8_t*)&num_routes);
		ext_flash_write((uint32_t)&eeprom_flash->routelist_list, sizeof(eeprom_flash->routelist_list), (uint8_t*)&routelist_list);
		ext_flash_write((uint32_t)&eeprom_flash->routememb_memb_mem, sizeof(eeprom_flash->routememb_memb_mem), (uint8_t*)&routememb_memb_mem[0]);
		ext_flash_write((uint32_t)&eeprom_flash->_nbr_routes_mem, sizeof(eeprom_flash->_nbr_routes_mem), (uint8_t*)&_nbr_routes_mem[0]);
		ext_flash_write((uint32_t)&eeprom_flash->defaultroutermemb_memb_mem, sizeof(eeprom_flash->defaultroutermemb_memb_mem), (uint8_t*)&defaultroutermemb_memb_mem[0]);
		ext_flash_write((uint32_t)&eeprom_flash->neighborroutememb_memb_count, sizeof(eeprom_flash->neighborroutememb_memb_count), (uint8_t*)&neighborroutememb_memb_count[0]);
		ext_flash_write((uint32_t)&eeprom_flash->neighborroutememb_memb_mem, sizeof(eeprom_flash->neighborroutememb_memb_mem), (uint8_t*)&neighborroutememb_memb_mem[0]);
		ext_flash_write((uint32_t)&eeprom_flash->routememb_memb_count, sizeof(eeprom_flash->routememb_memb_count), (uint8_t*)&routememb_memb_count[0]);

		/* CRC16 */
		uint16_t crc16;

		crc16 = crc16_add((uint8_t*)&defaultroutermemb, sizeof(eeprom_flash->defaultroutermemb), 0x0000);
		crc16 = crc16_add((uint8_t*)&nbr_routes, sizeof(eeprom_flash->nbr_routes), crc16);
		crc16 = crc16_add((uint8_t*)&nbr_routes_struct, sizeof(eeprom_flash->nbr_routes_struct), crc16);
		crc16 = crc16_add((uint8_t*)&neighborroutememb, sizeof(eeprom_flash->neighborroutememb), crc16);
		crc16 = crc16_add((uint8_t*)&routememb, sizeof(eeprom_flash->routememb), crc16);
		crc16 = crc16_add((uint8_t*)&defaultrouterlist_list, sizeof(eeprom_flash->defaultrouterlist_list), crc16);
		crc16 = crc16_add((uint8_t*)&defaultroutermemb_memb_count[0], sizeof(eeprom_flash->defaultroutermemb_memb_count), crc16);
		crc16 = crc16_add((uint8_t*)&notificationlist_list, sizeof(eeprom_flash->notificationlist_list), crc16);
		crc16 = crc16_add((uint8_t*)&num_routes, sizeof(eeprom_flash->num_routes), crc16);
		crc16 = crc16_add((uint8_t*)&routelist_list, sizeof(eeprom_flash->routelist_list), crc16);
		crc16 = crc16_add((uint8_t*)&routememb_memb_mem[0], sizeof(eeprom_flash->routememb_memb_mem), crc16);
		crc16 = crc16_add((uint8_t*)&_nbr_routes_mem[0], sizeof(eeprom_flash->_nbr_routes_mem), crc16);
		crc16 = crc16_add((uint8_t*)&defaultroutermemb_memb_mem[0], sizeof(eeprom_flash->defaultroutermemb_memb_mem), crc16);
		crc16 = crc16_add((uint8_t*)&neighborroutememb_memb_count[0], sizeof(eeprom_flash->neighborroutememb_memb_count), crc16);
		crc16 = crc16_add((uint8_t*)&neighborroutememb_memb_mem[0], sizeof(eeprom_flash->neighborroutememb_memb_mem), crc16);
		crc16 = crc16_add((uint8_t*)&routememb_memb_count[0], sizeof(eeprom_flash->routememb_memb_count), crc16);

		ext_flash_write((uint32_t)&eeprom_flash->crc16, sizeof(eeprom_flash->crc16), (uint8_t*)&crc16);
		PRINTF("crc16_route_table: 0x%04x\n", crc16);

		/* MAGIC BYTES */
		uint32_t magic_bytes = MAGIC_BYTES;
		ext_flash_write((uint32_t)&eeprom_flash->magic_bytes, sizeof(eeprom_flash->magic_bytes), (uint8_t*)&magic_bytes);

		ext_flash_close();
		return true;
	}
	else
	{
		PRINTF("save_routelist: could not access EEPROM\n");
		ext_flash_close();
		return false;
	}
}

/*---------------------------------------------------------------------------*/
/* Save routelist to EEPROM process */
PROCESS_THREAD(save_routelist_process, ev, data)
{
	PROCESS_BEGIN();
	
	if(ev == PROCESS_EVENT_EXIT)
		return 1;
	
	/* Создаём таймер для по истечении которого будет сохранятся таблица маршрутизации */
	static struct etimer save_routelist_timer;	
	
	while (1)
	{
		/* Устанавливаем таймер на срабатывание с раз в ROUTELIST_SAVE_INTERVAL */
		etimer_set(&save_routelist_timer, ROUTELIST_SAVE_INTERVAL);	

		/* Засыпаем до срабатывания таймера */
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&save_routelist_timer));

		// /* TEST PRINTF */		
		// printf("[DAG Node] Number of routes: %i\n", uip_ds6_route_num_routes());
		// if(save_routelist())
		// 	printf("[DAG Node] Save route table: ok\n");
		// else
		// 	printf("[DAG Node] Save route table: error\n");
		
		/* Если прошел 1ч и устройств > MIN_NUM_ROUTE_SAVE, то сохраняем таблицу маршрутизации */
		if(uip_ds6_route_num_routes() > MIN_NUM_ROUTE_SAVE)								
		{
			if(save_routelist())
				printf("save_routelist: ok\n");
			else
				printf("save_routelist: error\n");
		}
	}
	
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
#endif
/** @} */
