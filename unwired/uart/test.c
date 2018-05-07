Sizeof uip_ds6_route_t: 36
Sizeof uip_ds6_nbr_t: 18
Sizeof nbr_table_item_t: 1
Sizeof nbr_table_key_t: 12

/***************************************************************************/

/** \brief An entry in the routing table */
typedef struct uip_ds6_route {
  struct uip_ds6_route *next;
  /* Each route entry belongs to a specific neighbor. That neighbor
     holds a list of all routing entries that go through it. The
     routes field point to the uip_ds6_route_neighbor_routes that
     belong to the neighbor table entry that this routing table entry
     uses. */
  struct uip_ds6_route_neighbor_routes *neighbor_routes;
  uip_ipaddr_t ipaddr;
#ifdef UIP_DS6_ROUTE_STATE_TYPE
  UIP_DS6_ROUTE_STATE_TYPE state; //rpl_route_entry_t
#endif
  uint8_t length;
} uip_ds6_route_t;

/***************************************************************************/

/** \brief The neighbor routes hold a list of routing table entries
    that are attached to a specific neihbor. */
struct uip_ds6_route_neighbor_routes {
  LIST_STRUCT(route_list);
};

/***************************************************************************/

/**
 * Representation of an IP address.
 *
 */
typedef union uip_ip4addr_t {
  uint8_t  u8[4];                       /* Initializer, must come first. */
  uint16_t u16[2];
} uip_ip4addr_t;

typedef union uip_ip6addr_t {
  uint8_t  u8[16];                      /* Initializer, must come first. */
  uint16_t u16[8];
} uip_ip6addr_t;

#if NETSTACK_CONF_WITH_IPV6
typedef uip_ip6addr_t uip_ipaddr_t;
#else /* NETSTACK_CONF_WITH_IPV6 */
typedef uip_ip4addr_t uip_ipaddr_t;
#endif /* NETSTACK_CONF_WITH_IPV6 */

/***************************************************************************/

typedef struct __attribute__((__packed__)) rpl_route_entry {
  uint32_t lifetime;
  struct rpl_dag *dag;
  uint8_t dao_seqno_out;
  uint8_t dao_seqno_in;
  uint8_t state_flags;
} rpl_route_entry_t;

/***************************************************************************/

NBR_TABLE_GLOBAL(struct uip_ds6_route_neighbor_routes, nbr_routes);

/** \brief A non-static neighbor table. To be initialized through nbr_table_register(name) */
#define NBR_TABLE_GLOBAL(type, name) \
  static type _##name##_mem[NBR_TABLE_MAX_NEIGHBORS]; \
  static nbr_table_t name##_struct = { 0, sizeof(type), NULL, (nbr_table_item_t *)_##name##_mem }; \
  nbr_table_t *name = &name##_struct \

MEMB(neighborroutememb, struct uip_ds6_route_neighbor_route, UIP_DS6_ROUTE_NB);
  
#define MEMB(name, structure, num) \
        static char CC_CONCAT(name,_memb_count)[num]; \
        static structure CC_CONCAT(name,_memb_mem)[num]; \
        static struct memb name = {sizeof(structure), num, \
                                          CC_CONCAT(name,_memb_count), \
                                          (void *)CC_CONCAT(name,_memb_mem)}




/***************************************************************************/

https://e2e.ti.com/support/wireless_connectivity/proprietary_sub_1_ghz_simpliciti/f/156/t/677370?Compiler-CC1310-Using-the-Cache-as-GPRAM-for-application
https://github.com/contiki-os/contiki/wiki/RPL-modes


address: show net address	
	ADDRESS: not join to net, local address not available
	ADDRESS: node full ipv6 address: FD00::0212:4B00:0C46:8905
	ADDRESS: unwired net address: 02124B000C468905
	//Link layer addr: 00:12:4b:00:0c:46:7c:81
	//Node UD address: 02124B000C467C81
	FD00::0212:4B00:0C46:8A86

status: show node status
	STATUS: rpl parent ip address: FE80::0212:4B00:0C46:8905
	STATUS: rpl dag root ip address: FD00::0212:4B00:0C46:8905
	STATUS: rpl parent last tx: 6 sec ago
	STATUS: rpl parent rssi: -53
	STATUS: rpl parent is reachable: 1
	STATUS: temp: 21C, voltage: 3320mv


00:12:4B:00:0C:46:8A:86 Зашитый в чипе
 Link layer addr: 00:12:4b:00:0c:46:8a:86 Он же
02:12:4B:00:0C:46:8A:86 EUID64????
DAG-root node: FD00::0212:4B00:0C46:8A86 IPv6

00:12:4B - TexasIns	Texas Instruments

 Link layer addr: 00:12:4b:00:0c:46:8d:03
 Node UD address: 02124B000C468D03
	
	
void rpl_initialize()
{
	//printf("rpl_initialize\n");
   /* Set MESH-mode for dc-power rpl-root(not leaf-mode) */
   rpl_set_mode(RPL_MODE_MESH);

   static uip_ipaddr_t ipaddr;

   /* Fill in the address with zeros and the local prefix */
   uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);

   /* Generate an address based on the chip ID */
   uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);

   /* Adding autoconfigured address as the device address */
   uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

   /* make local address as rpl-root */
   rpl_set_root(RPL_DEFAULT_INSTANCE, &ipaddr);
   rpl_dag_t *dag = rpl_get_any_dag();

   uip_ipaddr_t prefix;
   uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
   rpl_set_prefix(dag, &prefix, 64);

   if(uart_status_r() == 0)
	 printf("UDM: Created a new RPL DAG, i'm root!\n");
 
   if(uart_status_r() == 0)
     printf("UDM: Time sync needed\n");
}


//fd00:0000:0000:0000:0212:4b00:0c46:7a01
//uip_ipaddr_t addr = { 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x12, 0x4b, 0x00, 0x0c, 0x46, 0x7a, 0x01 };





ext_addr: 00:12:4B:00:0C:46:8A:86
linkaddr_node_addr: 00:12:4B:00:0C:46:8A:86
short_addr: 8A:86
LINKADDR_SIZE: 8 
Channel: 26
Link layer addr: 00:12:4B:00:0C:46:8A:86
Node UD address: 02:12:4B:00:0C:46:8A:86
Max routes: 100 (UIP_CONF_MAX_ROUTES)

UIP_DS6_DEFAULT_PREFIX: FD00

NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, IEEE802154_PANID);
NETSTACK_RADIO.set_value(RADIO_PARAM_16BIT_ADDR, short_addr);
NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, RF_CORE_CHANNEL);
NETSTACK_RADIO.set_object(RADIO_PARAM_64BIT_ADDR, ext_addr, 8);

uip_ds6_neighbors_init();
uip_ds6_route_init();

/** \name Routing Table basic routines */
/** @{ */
uip_ds6_route_t *uip_ds6_route_lookup(uip_ipaddr_t *destipaddr);
uip_ds6_route_t *uip_ds6_route_add(uip_ipaddr_t *ipaddr, uint8_t length,
                                   uip_ipaddr_t *next_hop);
void uip_ds6_route_rm(uip_ds6_route_t *route);
void uip_ds6_route_rm_by_nexthop(uip_ipaddr_t *nexthop);
uip_ipaddr_t *uip_ds6_route_nexthop(uip_ds6_route_t *);
int uip_ds6_route_num_routes(void);
uip_ds6_route_t *uip_ds6_route_head(void);
uip_ds6_route_t *uip_ds6_route_next(uip_ds6_route_t *);
int uip_ds6_route_is_nexthop(const uip_ipaddr_t *ipaddr);
/** @} */

/* The nbr_routes holds a neighbor table to be able to maintain
   information about what routes go through what neighbor. This
   neighbor table is registered with the central nbr-table repository
   so that it will be maintained along with the rest of the neighbor
   tables in the system. */
NBR_TABLE_GLOBAL(struct uip_ds6_route_neighbor_routes, nbr_routes);
MEMB(neighborroutememb, struct uip_ds6_route_neighbor_route, UIP_DS6_ROUTE_NB);

/* Each route is repressented by a uip_ds6_route_t structure and
   memory for each route is allocated from the routememb memory
   block. These routes are maintained on the routelist. */
LIST(routelist);
MEMB(routememb, uip_ds6_route_t, UIP_DS6_ROUTE_NB);

static int num_routes = 0;
static void rm_routelist_callback(nbr_table_item_t *ptr);


/** \brief A static neighbor table. To be initialized through nbr_table_register(name) */
#define NBR_TABLE(type, name) \
  static type _##name##_mem[NBR_TABLE_MAX_NEIGHBORS]; \
  static nbr_table_t name##_struct = { 0, sizeof(type), NULL, (nbr_table_item_t *)_##name##_mem }; \
  static nbr_table_t *name = &name##_struct \

/** \brief A non-static neighbor table. To be initialized through nbr_table_register(name) */
#define NBR_TABLE_GLOBAL(type, name) \
  static type _##name##_mem[NBR_TABLE_MAX_NEIGHBORS]; \
  static nbr_table_t name##_struct = { 0, sizeof(type), NULL, (nbr_table_item_t *)_##name##_mem }; \
  nbr_table_t *name = &name##_struct \

/** \brief Declaration of non-static neighbor tables */
#define NBR_TABLE_DECLARE(name) extern nbr_table_t *name

/* RPL routing table functions. */
void rpl_remove_routes(rpl_dag_t *dag);
void rpl_remove_routes_by_nexthop(uip_ipaddr_t *nexthop, rpl_dag_t *dag);
uip_ds6_route_t *rpl_add_route(rpl_dag_t *dag, uip_ipaddr_t *prefix,
                               int prefix_len, uip_ipaddr_t *next_hop);
void rpl_purge_routes(void);

NBR_TABLE_GLOBAL(uip_ds6_nbr_t, ds6_neighbors);

/*---------------------------------------------------------------------------*/
uip_ds6_route_t *
uip_ds6_route_lookup(uip_ipaddr_t *addr)
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
      r = uip_ds6_route_next(r)) {
    if(r->length >= longestmatch &&
       uip_ipaddr_prefixcmp(addr, &r->ipaddr, r->length)) {
      longestmatch = r->length;
      found_route = r;
      /* check if total match - e.g. all 128 bits do match */
      if(longestmatch == 128) {
	break;
      }
    }
  }

  if(found_route != NULL) {
    PRINTF("uip-ds6-route: Found route: ");
    PRINT6ADDR(addr);
    PRINTF(" via ");
    PRINT6ADDR(uip_ds6_route_nexthop(found_route));
    PRINTF("\n");
  } else {
    PRINTF("uip-ds6-route: No route found\n");
  }

  if(found_route != NULL && found_route != list_head(routelist)) {
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













static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ
static void uart_to_air() //ЧО ЗА ХУЙНЯ НАДО ПЕРЕДЕЛАТЬ И ПРЕОТЛАДИТЬ



/*---------------------------------------------------------------------------*/
uip_ds6_route_t *
uip_ds6_route_add(uip_ipaddr_t *ipaddr, uint8_t length,
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
  if(nexthop_lladdr == NULL) {
    PRINTF("uip_ds6_route_add: neighbor link-local address unknown for ");
    PRINT6ADDR(nexthop);
    PRINTF("\n");
    return NULL;
  }

  /* First make sure that we don't add a route twice. If we find an
     existing route for our destination, we'll delete the old
     one first. */
  r = uip_ds6_route_lookup(ipaddr);
  if(r != NULL) {
    uip_ipaddr_t *current_nexthop;
    current_nexthop = uip_ds6_route_nexthop(r);
    if(current_nexthop != NULL && uip_ipaddr_cmp(nexthop, current_nexthop)) {
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

    if(uip_ds6_route_num_routes() == UIP_DS6_ROUTE_NB) {
      uip_ds6_route_t *oldest;
      oldest = NULL;
#if UIP_DS6_ROUTE_REMOVE_LEAST_RECENTLY_USED
      /* Removing the oldest route entry from the route table. The
         least recently used route is the first route on the list. */
      oldest = list_tail(routelist);
#endif
      if(oldest == NULL) {
        return NULL;
      }
      PRINTF("uip_ds6_route_add: dropping route to ");
      PRINT6ADDR(&oldest->ipaddr);
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
    routes = nbr_table_get_from_lladdr(nbr_routes,
                                       (linkaddr_t *)nexthop_lladdr);

    if(routes == NULL) {
      /* If the neighbor did not have an entry in our neighbor table,
         we create one. The nbr_table_add_lladdr() function returns a
         pointer to a pointer that we may use for our own purposes. We
         initialize this pointer with the list of routing entries that
         are attached to this neighbor. */
      routes = nbr_table_add_lladdr(nbr_routes,
                                    (linkaddr_t *)nexthop_lladdr,
                                    NBR_TABLE_REASON_ROUTE, NULL);
      if(routes == NULL) {
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

    if(r == NULL) {
      /* This should not happen, as we explicitly deallocated one
         route table entry above. */
      PRINTF("uip_ds6_route_add: could not allocate route\n");
      return NULL;
    }

    /* add new routes first - assuming that there is a reason to add this
       and that there is a packet coming soon. */
    list_push(routelist, r);

    nbrr = memb_alloc(&neighborroutememb);
    if(nbrr == NULL) {
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

  uip_ipaddr_copy(&(r->ipaddr), ipaddr);
  r->length = length;

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

/*---------------------------------------------------------------------------*/  
uip_ds6_route_t *
uip_ds6_route_add(uip_ipaddr_t *ipaddr, uint8_t length,
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
  if(nexthop_lladdr == NULL) {
    PRINTF("uip_ds6_route_add: neighbor link-local address unknown for ");
    PRINT6ADDR(nexthop);
    PRINTF("\n");
    return NULL;
  }

  /* First make sure that we don't add a route twice. If we find an
     existing route for our destination, we'll delete the old
     one first. */
  r = uip_ds6_route_lookup(ipaddr);
  if(r != NULL) {
    uip_ipaddr_t *current_nexthop;
    current_nexthop = uip_ds6_route_nexthop(r);
    if(current_nexthop != NULL && uip_ipaddr_cmp(nexthop, current_nexthop)) {
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

    if(uip_ds6_route_num_routes() == UIP_DS6_ROUTE_NB) {
      uip_ds6_route_t *oldest;
      oldest = NULL;
#if UIP_DS6_ROUTE_REMOVE_LEAST_RECENTLY_USED
      /* Removing the oldest route entry from the route table. The
         least recently used route is the first route on the list. */
      oldest = list_tail(routelist);
#endif
      if(oldest == NULL) {
        return NULL;
      }
      PRINTF("uip_ds6_route_add: dropping route to ");
      PRINT6ADDR(&oldest->ipaddr);
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
    routes = nbr_table_get_from_lladdr(nbr_routes,
                                       (linkaddr_t *)nexthop_lladdr);

    if(routes == NULL) {
      /* If the neighbor did not have an entry in our neighbor table,
         we create one. The nbr_table_add_lladdr() function returns a
         pointer to a pointer that we may use for our own purposes. We
         initialize this pointer with the list of routing entries that
         are attached to this neighbor. */
      routes = nbr_table_add_lladdr(nbr_routes,
                                    (linkaddr_t *)nexthop_lladdr,
                                    NBR_TABLE_REASON_ROUTE, NULL);
      if(routes == NULL) {
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

    if(r == NULL) {
      /* This should not happen, as we explicitly deallocated one
         route table entry above. */
      PRINTF("uip_ds6_route_add: could not allocate route\n");
      return NULL;
    }

    /* add new routes first - assuming that there is a reason to add this
       and that there is a packet coming soon. */
    list_push(routelist, r);

    nbrr = memb_alloc(&neighborroutememb);
    if(nbrr == NULL) {
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

  uip_ipaddr_copy(&(r->ipaddr), ipaddr);
  r->length = length;

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
	nbr_table_item_t 
nbr_table_key_t 
uip_ds6_route_t
	uip_ipaddr_t 
	rpl_dag_t 
rpl_route_entry_t
nbr_table_t
linkaddr_t
nbr_table_reason_t 

typedef struct nbr_table_key {
  struct nbr_table_key *next;
  linkaddr_t lladdr;
} nbr_table_key_t;


/** \brief An entry in the routing table */
typedef struct uip_ds6_route { 
  struct uip_ds6_route *next;
  /* Each route entry belongs to a specific neighbor. That neighbor
     holds a list of all routing entries that go through it. The
     routes field point to the uip_ds6_route_neighbor_routes that
     belong to the neighbor table entry that this routing table entry
     uses. */
  struct uip_ds6_route_neighbor_routes *neighbor_routes;
  uip_ipaddr_t ipaddr;
#ifdef UIP_DS6_ROUTE_STATE_TYPE
  UIP_DS6_ROUTE_STATE_TYPE state;
#endif
  uint8_t length;
} uip_ds6_route_t;

typedef struct __attribute__((__packed__)) rpl_route_entry {
  uint32_t lifetime;
  struct rpl_dag *dag;
  uint8_t dao_seqno_out;
  uint8_t dao_seqno_in;
  uint8_t state_flags;
} rpl_route_entry_t;

typedef struct nbr_table {
	int index;
	int item_size;
	nbr_table_callback *callback;
	nbr_table_item_t *data;
} nbr_table_t;
	
/**
 * \brief      The Rime address of the node
 *
 *             This variable contains the Rime address of the
 *             node. This variable should not be changed directly;
 *             rather, the linkaddr_set_node_addr() function should be
 *             used.
 *
 */
extern linkaddr_t linkaddr_node_addr;
	
typedef enum {
	NBR_TABLE_REASON_UNDEFINED,
	NBR_TABLE_REASON_RPL_DIO,
	NBR_TABLE_REASON_RPL_DAO,
	NBR_TABLE_REASON_RPL_DIS,
	NBR_TABLE_REASON_ROUTE,
	NBR_TABLE_REASON_IPV6_ND,
	NBR_TABLE_REASON_MAC,
	NBR_TABLE_REASON_LLSEC,
	NBR_TABLE_REASON_LINK_STATS,
} nbr_table_reason_t;

/** \brief The neighbor routes hold a list of routing table entries
    that are attached to a specific neihbor. */
struct uip_ds6_route_neighbor_routes {
  LIST_STRUCT(route_list);
};

/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *uip_ds6_nbr_add( const uip_ipaddr_t *ipaddr,
								const uip_lladdr_t *lladdr,
								uint8_t isrouter, 
								uint8_t state, 
								nbr_table_reason_t reason,
								void *data)
{
	uip_ds6_nbr_t *nbr = nbr_table_add_lladdr ( ds6_neighbors,
												(linkaddr_t*)lladdr, 
												reason, 
												data);
}

/* Add a neighbor indexed with its link-layer address */
nbr_table_item_t *nbr_table_add_lladdr( nbr_table_t *table,
										const linkaddr_t *lladdr, 
										nbr_table_reason_t reason, 
										void *data)
{
  int index;
  nbr_table_item_t *item;
  nbr_table_key_t *key;

  /* Allow lladdr-free insertion, useful e.g. for IPv6 ND.
   * Only one such entry is possible at a time, indexed by linkaddr_null. */
  if(lladdr == NULL) {
    lladdr = &linkaddr_null;
  }

  if((index = index_from_lladdr(lladdr)) == -1) {
     /* Neighbor not yet in table, let's try to allocate one */
    key = nbr_table_allocate(reason, data);

    /* No space available for new entry */
    if(key == NULL) {
      return NULL;
    }

    /* Add neighbor to list */
    list_add(nbr_table_keys, key);

    /* Get index from newly allocated neighbor */
    index = index_from_key(key);

    /* Set link-layer address */
    linkaddr_copy(&key->lladdr, lladdr);
  }

  /* Get item in the current table */
  item = item_from_index(table, index);

  /* Initialize item data and set "used" bit */
  memset(item, 0, table->item_size);
  nbr_set_bit(used_map, table, item, 1);

#if DEBUG
  print_table();
#endif
  return item;
}

/*---------------------------------------------------------------------------*/
Bootloader:	 SPI flash not found, jump to main image
Starting Contiki 3.x With DriverLib v0.47020

Unwired Devices udboards/CC2650 7x7, version: v0.63-132-gc172dcf-dirty
Build on: Apr 27 2018 17:33:09
IEEE 802.15.4: Yes, CC13xx: No
Channel: 26
Link layer addr: 00:12:4b:00:0c:46:8a:86
Node UD address: 02124B000C468A86
PAN ID: 0xAABB
RPL probing interval: 5h(300m)
Max routes: 100
Init of IPv6 data structures
20 neighbors
2 default routers
3 prefixes
100 routes
3 unicast addresses
5 multicast addresses
2 anycast addresses
Adding prefix FE80::length 64, flags 0, Valid lifetime 0, Preffered lifetime 0
RPL started
Start Unwired RLP root.
RPL: switching to mesh mode
RPL: Node set to be a DAG root with DAG ID FD00::0212:4B00:0C46:8A86
RPL: Scheduling DIO timer 518 ticks in future (Interval)
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NULL
UDM: Created a new RPL DAG, i'm root!
UDM: Time sync needed
RPL: DIO Timer triggered
RPL: Sending prefix info in DIO for FD00::
RPL: Sending a multicast-DIO with rank 128
RPL: Scheduling DIO timer 6 ticks in future (sent)
RPL: MOP 2 OCP 1 rank 128 dioint 12, nbr count 0
RPL: end of list
RPL: DIO Timer triggered
RPL: DIO Timer interval doubled 13
RPL: Scheduling DIO timer 794 ticks in future (Interval)
RPL: MOP 2 OCP 1 rank 128 dioint 13, nbr count 0
RPL: end of list
NBR TABLE:
 00 03 [1:0] [0:0] [0:0] [0:0]
RPL: Received a DIS from FE80::0212:4B00:0C46:8D03
RPL: Multicast DIS => reset DIO timer
RPL: Scheduling DIO timer 436 ticks in future (Interval)
RPL: DIO Timer triggered
RPL: Sending prefix info in DIO for FD00::
RPL: Sending a multicast-DIO with rank 128
RPL: Scheduling DIO timer 88 ticks in future (sent)
RPL: MOP 2 OCP 1 rank 128 dioint 12, nbr count 0
RPL: end of list
RPL: DIO Timer triggered
RPL: DIO Timer interval doubled 13
RPL: Scheduling DIO timer 713 ticks in future (Interval)
RPL: MOP 2 OCP 1 rank 128 dioint 13, nbr count 0
RPL: end of list
RPL: Packet going up, sender closer 0 (384 < 128)
RPL: Rank OK
RPL: Creating hop-by-hop option
RPL: Updating RPL option
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: No route found
RPL option going up
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: No route found
RPL: Received a DIO from FE80::0212:4B00:0C46:8D03
RPL: Incoming DIO (id, ver, rank) = (30,240,345)
RPL: Incoming DIO (dag_id, pref) = (FD00::0212:4B00:0C46:8A86, 0)
RPL: DIO option 4, length: 14
RPL: DAG conf:dbl=4, min=12 red=10 maxinc=896 mininc=128 ocp=1 d_l=10 l_u=3600
RPL: DIO option 8, length: 30
RPL: Copying prefix information
RPL: Prefix announced in DIO
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NON-NULL
NBR TABLE:
 00 03 [1:0] [1:0] [0:0] [0:0]
Adding neighbor with ip addr FE80::0212:4B00:0C46:8D03 link addr 00:12:4b:00:0c:46:8d:03 state 1
RPL: Neighbor state changed for FE80::0212:4B00:0C46:8D03, state=1
RPL: Neighbor added to neighbor cache FE80::0212:4B00:0C46:8D03, 00:12:4b:00:0c:46:8d:03
RPL: Received a DAO from FE80::0212:4B00:0C46:8D03
RPL: Received a (unicast) DAO with sequence number 241 from FE80::0212:4B00:0C46:8D03
RPL: DAO lifetime: 10, prefix length: 128 prefix: FD00::0212:4B00:0C46:8D03
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: No route found
RPL: Adding DAO route
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: No route found
NBR TABLE:
 00 03 [1:0] [1:0] [1:0] [0:0]
uip_ds6_route_add num 1
*** Lock 0
uip_ds6_route_add: adding route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Added a route to FD00::0212:4B00:0C46:8D03/128 via FE80::0212:4B00:0C46:8D03
RPL: Packet going up, sender closer 0 (318 < 128)
RPL: Rank OK
RPL: Creating hop-by-hop option
RPL: Updating RPL option
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL option going down
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Packet going up, sender closer 0 (318 < 128)
RPL: Rank OK
RPL: Creating hop-by-hop option
RPL: Updating RPL option
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL option going down
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Packet going up, sender closer 0 (318 < 128)
RPL: Rank OK
RPL: Creating hop-by-hop option
RPL: Updating RPL option
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL option going down
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
16050002124B000C468D031A9B0012000002124B000C468A860A00000016003F007BB1
RPL: DIO Timer triggered
RPL: Sending prefix info in DIO for FD00::
RPL: Sending a multicast-DIO with rank 128
RPL: Scheduling DIO timer 335 ticks in future (sent)
RPL: MOP 2 OCP 1 rank 128 dioint 13, nbr count 1
RPL: end of list
RPL: Received a DIO from FE80::0212:4B00:0C46:8D03
RPL: Incoming DIO (id, ver, rank) = (30,240,286)
RPL: Incoming DIO (dag_id, pref) = (FD00::0212:4B00:0C46:8A86, 0)
RPL: DIO option 4, length: 14
RPL: DAG conf:dbl=4, min=12 red=10 maxinc=896 mininc=128 ocp=1 d_l=10 l_u=3600
RPL: DIO option 8, length: 30
RPL: Copying prefix information
RPL: Prefix announced in DIO
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NON-NULL
RPL: Received a DIO from FE80::0212:4B00:0C46:8D03
RPL: Incoming DIO (id, ver, rank) = (30,240,281)
RPL: Incoming DIO (dag_id, pref) = (FD00::0212:4B00:0C46:8A86, 0)
RPL: DIO option 4, length: 14
RPL: DAG conf:dbl=4, min=12 red=10 maxinc=896 mininc=128 ocp=1 d_l=10 l_u=3600
RPL: DIO option 8, length: 30
RPL: Copying prefix information
RPL: Prefix announced in DIO
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NON-NULL
RPL: DIO Timer triggered
RPL: DIO Timer interval doubled 14
RPL: Scheduling DIO timer 1164 ticks in future (Interval)
RPL: MOP 2 OCP 1 rank 128 dioint 14, nbr count 1
RPL: end of list
RPL: Received a DAO from FE80::0212:4B00:0C46:8D03
RPL: Received a (unicast) DAO with sequence number 242 from FE80::0212:4B00:0C46:8D03
RPL: DAO lifetime: 10, prefix length: 128 prefix: FD00::0212:4B00:0C46:8D03
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Adding DAO route
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Added a route to FD00::0212:4B00:0C46:8D03/128 via FE80::0212:4B00:0C46:8D03
RPL: DIO Timer triggered
RPL: Sending prefix info in DIO for FD00::
RPL: Sending a multicast-DIO with rank 128
RPL: Scheduling DIO timer 933 ticks in future (sent)
RPL: MOP 2 OCP 1 rank 128 dioint 14, nbr count 1
RPL: end of list
UDM: UART change to alt(RX: 26, TX: 25)
RPL: Received a DAO from FE80::0212:4B00:0C46:8D03
RPL: Received a (unicast) DAO with sequence number 243 from FE80::0212:4B00:0C46:8D03
RPL: DAO lifetime: 10, prefix length: 128 prefix: FD00::0212:4B00:0C46:8D03
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Adding DAO route
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Added a route to FD00::0212:4B00:0C46:8D03/128 via FE80::0212:4B00:0C46:8D03
RPL: DIO Timer triggered
RPL: DIO Timer interval doubled 15
RPL: Scheduling DIO timer 3135 ticks in future (Interval)
RPL: MOP 2 OCP 1 rank 128 dioint 15, nbr count 1
RPL: end of list
RPL: Received a DIO from FE80::0212:4B00:0C46:8D03
RPL: Incoming DIO (id, ver, rank) = (30,240,273)
RPL: Incoming DIO (dag_id, pref) = (FD00::0212:4B00:0C46:8A86, 0)
RPL: DIO option 4, length: 14
RPL: DAG conf:dbl=4, min=12 red=10 maxinc=896 mininc=128 ocp=1 d_l=10 l_u=3600
RPL: DIO option 8, length: 30
RPL: Copying prefix information
RPL: Prefix announced in DIO
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NON-NULL
RPL: Received a DIO from FE80::0212:4B00:0C46:8D03
RPL: Incoming DIO (id, ver, rank) = (30,240,273)
RPL: Incoming DIO (dag_id, pref) = (FD00::0212:4B00:0C46:8A86, 0)
RPL: DIO option 4, length: 14
RPL: DAG conf:dbl=4, min=12 red=10 maxinc=896 mininc=128 ocp=1 d_l=10 l_u=3600
RPL: DIO option 8, length: 30
RPL: Copying prefix information
RPL: Prefix announced in DIO
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NON-NULL
NBR TABLE:
 00 03 [1:0] [1:0] [1:1] [0:0]
RPL: DIO Timer triggered
RPL: Sending prefix info in DIO for FD00::
RPL: Sending a multicast-DIO with rank 128
RPL: Scheduling DIO timer 1059 ticks in future (sent)
RPL: MOP 2 OCP 1 rank 128 dioint 15, nbr count 1
RPL: end of list
RPL: Received a DAO from FE80::0212:4B00:0C46:8D03
RPL: Received a (unicast) DAO with sequence number 244 from FE80::0212:4B00:0C46:8D03
RPL: DAO lifetime: 10, prefix length: 128 prefix: FD00::0212:4B00:0C46:8D03
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Adding DAO route
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Added a route to FD00::0212:4B00:0C46:8D03/128 via FE80::0212:4B00:0C46:8D03
RPL: DIO Timer triggered
RPL: DIO Timer interval doubled 16
RPL: Scheduling DIO timer 7086 ticks in future (Interval)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
NBR TABLE:
 00 03 [1:0] [1:0] [1:1] [0:0]
RPL: DIO Timer triggered
RPL: Sending prefix info in DIO for FD00::
RPL: Sending a multicast-DIO with rank 128
RPL: Scheduling DIO timer 1302 ticks in future (sent)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
RPL: Received a DIO from FE80::0212:4B00:0C46:8D03
RPL: Incoming DIO (id, ver, rank) = (30,240,289)
RPL: Incoming DIO (dag_id, pref) = (FD00::0212:4B00:0C46:8A86, 0)
RPL: DIO option 4, length: 14
RPL: DAG conf:dbl=4, min=12 red=10 maxinc=896 mininc=128 ocp=1 d_l=10 l_u=3600
RPL: DIO option 8, length: 30
RPL: Copying prefix information
RPL: Prefix announced in DIO
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NON-NULL
RPL: Received a DAO from FE80::0212:4B00:0C46:8D03
RPL: Received a (unicast) DAO with sequence number 245 from FE80::0212:4B00:0C46:8D03
RPL: DAO lifetime: 10, prefix length: 128 prefix: FD00::0212:4B00:0C46:8D03
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Adding DAO route
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Added a route to FD00::0212:4B00:0C46:8D03/128 via FE80::0212:4B00:0C46:8D03
RPL: DIO Timer triggered
RPL: Scheduling DIO timer 4879 ticks in future (Interval)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
RPL: DIO Timer triggered
RPL: Sending prefix info in DIO for FD00::
RPL: Sending a multicast-DIO with rank 128
RPL: Scheduling DIO timer 3509 ticks in future (sent)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
RPL: Received a DAO from FE80::0212:4B00:0C46:8D03
RPL: Received a (unicast) DAO with sequence number 246 from FE80::0212:4B00:0C46:8D03
RPL: DAO lifetime: 10, prefix length: 128 prefix: FD00::0212:4B00:0C46:8D03
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Adding DAO route
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Added a route to FD00::0212:4B00:0C46:8D03/128 via FE80::0212:4B00:0C46:8D03
NBR TABLE:
 00 03 [1:0] [1:0] [1:1] [0:0]
RPL: DIO Timer triggered
RPL: Scheduling DIO timer 4288 ticks in future (Interval)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
RPL: Received a DIO from FE80::0212:4B00:0C46:8D03
RPL: Incoming DIO (id, ver, rank) = (30,240,279)
RPL: Incoming DIO (dag_id, pref) = (FD00::0212:4B00:0C46:8A86, 0)
RPL: DIO option 4, length: 14
RPL: DAG conf:dbl=4, min=12 red=10 maxinc=896 mininc=128 ocp=1 d_l=10 l_u=3600
RPL: DIO option 8, length: 30
RPL: Copying prefix information
RPL: Prefix announced in DIO
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NON-NULL
RPL: DIO Timer triggered
RPL: Sending prefix info in DIO for FD00::
RPL: Sending a multicast-DIO with rank 128
RPL: Scheduling DIO timer 4100 ticks in future (sent)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
RPL: Received a DAO from FE80::0212:4B00:0C46:8D03
RPL: Received a (unicast) DAO with sequence number 247 from FE80::0212:4B00:0C46:8D03
RPL: DAO lifetime: 10, prefix length: 128 prefix: FD00::0212:4B00:0C46:8D03
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Adding DAO route
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Added a route to FD00::0212:4B00:0C46:8D03/128 via FE80::0212:4B00:0C46:8D03
NBR TABLE:
 00 03 [1:0] [1:0] [1:1] [0:0]
RPL: Received a DIO from FE80::0212:4B00:0C46:8D03
RPL: Incoming DIO (id, ver, rank) = (30,240,275)
RPL: Incoming DIO (dag_id, pref) = (FD00::0212:4B00:0C46:8A86, 0)
RPL: DIO option 4, length: 14
RPL: DAG conf:dbl=4, min=12 red=10 maxinc=896 mininc=128 ocp=1 d_l=10 l_u=3600
RPL: DIO option 8, length: 30
RPL: Copying prefix information
RPL: Prefix announced in DIO
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NON-NULL
RPL: DIO Timer triggered
RPL: Scheduling DIO timer 6055 ticks in future (Interval)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
NBR TABLE:
 00 03 [1:0] [1:0] [1:1] [0:0]
RPL: Received a DIO from FE80::0212:4B00:0C46:8D03
RPL: Incoming DIO (id, ver, rank) = (30,240,275)
RPL: Incoming DIO (dag_id, pref) = (FD00::0212:4B00:0C46:8A86, 0)
RPL: DIO option 4, length: 14
RPL: DAG conf:dbl=4, min=12 red=10 maxinc=896 mininc=128 ocp=1 d_l=10 l_u=3600
RPL: DIO option 8, length: 30
RPL: Copying prefix information
RPL: Prefix announced in DIO
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NON-NULL
RPL: DIO Timer triggered
RPL: Sending prefix info in DIO for FD00::
RPL: Sending a multicast-DIO with rank 128
RPL: Scheduling DIO timer 2333 ticks in future (sent)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
RPL: Received a DAO from FE80::0212:4B00:0C46:8D03
RPL: Received a (unicast) DAO with sequence number 248 from FE80::0212:4B00:0C46:8D03
RPL: DAO lifetime: 10, prefix length: 128 prefix: FD00::0212:4B00:0C46:8D03
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Adding DAO route
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Added a route to FD00::0212:4B00:0C46:8D03/128 via FE80::0212:4B00:0C46:8D03
RPL: DIO Timer triggered
RPL: Scheduling DIO timer 4996 ticks in future (Interval)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
NBR TABLE:
 00 03 [1:0] [1:0] [1:1] [0:0]
RPL: DIO Timer triggered
RPL: Sending prefix info in DIO for FD00::
RPL: Sending a multicast-DIO with rank 128
RPL: Scheduling DIO timer 3392 ticks in future (sent)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
RPL: Received a DIO from FE80::0212:4B00:0C46:8D03
RPL: Incoming DIO (id, ver, rank) = (30,240,272)
RPL: Incoming DIO (dag_id, pref) = (FD00::0212:4B00:0C46:8A86, 0)
RPL: DIO option 4, length: 14
RPL: DAG conf:dbl=4, min=12 red=10 maxinc=896 mininc=128 ocp=1 d_l=10 l_u=3600
RPL: DIO option 8, length: 30
RPL: Copying prefix information
RPL: Prefix announced in DIO
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NON-NULL
RPL: Received a DAO from FE80::0212:4B00:0C46:8D03
RPL: Received a (unicast) DAO with sequence number 249 from FE80::0212:4B00:0C46:8D03
RPL: DAO lifetime: 10, prefix length: 128 prefix: FD00::0212:4B00:0C46:8D03
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Adding DAO route
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Added a route to FD00::0212:4B00:0C46:8D03/128 via FE80::0212:4B00:0C46:8D03
RPL: DIO Timer triggered
RPL: Scheduling DIO timer 6264 ticks in future (Interval)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
NBR TABLE:
 00 03 [1:0] [1:0] [1:1] [0:0]
RPL: Received a DIO from FE80::0212:4B00:0C46:8D03
RPL: Incoming DIO (id, ver, rank) = (30,240,269)
RPL: Incoming DIO (dag_id, pref) = (FD00::0212:4B00:0C46:8A86, 0)
RPL: DIO option 4, length: 14
RPL: DAG conf:dbl=4, min=12 red=10 maxinc=896 mininc=128 ocp=1 d_l=10 l_u=3600
RPL: DIO option 8, length: 30
RPL: Copying prefix information
RPL: Prefix announced in DIO
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NON-NULL
RPL: DIO Timer triggered
RPL: Sending prefix info in DIO for FD00::
RPL: Sending a multicast-DIO with rank 128
RPL: Scheduling DIO timer 2124 ticks in future (sent)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
RPL: Received a DAO from FE80::0212:4B00:0C46:8D03
RPL: Received a (unicast) DAO with sequence number 250 from FE80::0212:4B00:0C46:8D03
RPL: DAO lifetime: 10, prefix length: 128 prefix: FD00::0212:4B00:0C46:8D03
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Adding DAO route
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Added a route to FD00::0212:4B00:0C46:8D03/128 via FE80::0212:4B00:0C46:8D03
RPL: DIO Timer triggered
RPL: Scheduling DIO timer 5452 ticks in future (Interval)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
NBR TABLE:
 00 03 [1:0] [1:0] [1:1] [0:0]
RPL: Received a DIO from FE80::0212:4B00:0C46:8D03
RPL: Incoming DIO (id, ver, rank) = (30,240,267)
RPL: Incoming DIO (dag_id, pref) = (FD00::0212:4B00:0C46:8A86, 0)
RPL: DIO option 4, length: 14
RPL: DAG conf:dbl=4, min=12 red=10 maxinc=896 mininc=128 ocp=1 d_l=10 l_u=3600
RPL: DIO option 8, length: 30
RPL: Copying prefix information
RPL: Prefix announced in DIO
RPL: Prefix set - will announce this in DIOs
rpl_set_prefix - prefix NON-NULL
RPL: DIO Timer triggered
RPL: Sending prefix info in DIO for FD00::
RPL: Sending a multicast-DIO with rank 128
RPL: Scheduling DIO timer 2936 ticks in future (sent)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
RPL: Received a DAO from FE80::0212:4B00:0C46:8D03
RPL: Received a (unicast) DAO with sequence number 251 from FE80::0212:4B00:0C46:8D03
RPL: DAO lifetime: 10, prefix length: 128 prefix: FD00::0212:4B00:0C46:8D03
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Adding DAO route
uip-ds6-route: Looking up route for FD00::0212:4B00:0C46:8D03
uip-ds6-route: Found route: FD00::0212:4B00:0C46:8D03 via FE80::0212:4B00:0C46:8D03
RPL: Added a route to FD00::0212:4B00:0C46:8D03/128 via FE80::0212:4B00:0C46:8D03
RPL: DIO Timer triggered
RPL: Scheduling DIO timer 5381 ticks in future (Interval)
RPL: MOP 2 OCP 1 rank 128 dioint 16, nbr count 1
RPL: end of list
NBR TABLE:
 00 03 [1:0] [1:0] [1:1] [0:0]
