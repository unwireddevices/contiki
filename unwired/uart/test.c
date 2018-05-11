Sizeof uip_ds6_route_t: 36
Sizeof uip_ds6_nbr_t: 18
Sizeof nbr_table_item_t: 1
Sizeof nbr_table_key_t: 12
Sizeof nbr_table_t: 16

UIP_CONF_MAX_ROUTES
nbr_table_t
#define UIP_DS6_ROUTE_NB UIP_CONF_MAX_ROUTES
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
typedef union uip_ip6addr_compress_t {	//
  uint8_t  u8[8];                      	//
  uint16_t u16[4];						//
} uip_ip6addr_compress_t;				//

typedef union uip_ip6addr_t {
  uint8_t  u8[16];                      /* Initializer, must come first. */
  uint16_t u16[8];
} uip_ip6addr_t;

#if NETSTACK_CONF_WITH_IPV6
typedef uip_ip6addr_t uip_ipaddr_t;
#else /* NETSTACK_CONF_WITH_IPV6 */
typedef uip_ip4addr_t uip_ipaddr_t;
#endif /* NETSTACK_CONF_WITH_IPV6 */

void compress_uip_ipaddr_t(uip_ipaddr_t *addr_in, uip_ipaddr_compress_t *addr_out) 
{
	addr_out.u16[0] = addr_in[4];
	addr_out.u16[1] = addr_in[5];
	addr_out.u16[2] = addr_in[6];
	addr_out.u16[3] = addr_in[7];
}

void decompress_uip_ipaddr_t(uip_ipaddr_t *addr_out, uip_ipaddr_compress_t *addr_in)
{
	addr_out.u16[0] = 0xFD00;
	addr_out.u16[1] = 0;
	addr_out.u16[2] = 0;
	addr_out.u16[3] = 0;
	addr_out.u16[4] = addr_in[0];
	addr_out.u16[5] = addr_in[1];
	addr_out.u16[6] = addr_in[2];
	addr_out.u16[7] = addr_in[3];
}


uip_ip6addr_t find_addr(uint32_t serial)
{
	uip_ip6addr_t addr;
	
	for(uint8_t i = 0; i < route_table_ptr; i++)
	{
		if(route_table[i].serial == serial)
		{
			if(route_table[i].counter != 0xFFFF) //Проверка на активность.
			{
				uip_ip6addr(&addr,  
							0xFD00,
							0,
							0,
							0,
							(uint16_t)(route_table[i].addr[1] | (route_table[i].addr[0]<<8)),
							(uint16_t)(route_table[i].addr[3] | (route_table[i].addr[2]<<8)),
							(uint16_t)(route_table[i].addr[5] | (route_table[i].addr[4]<<8)),
							(uint16_t)(route_table[i].addr[7] | (route_table[i].addr[6]<<8)));
							
				return addr;
			}
		}
	}
	
	uip_ip6addr(&addr, 0, 0, 0, 0, 0, 0, 0, 0); //Адрес не найден
	return addr;
}




















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


/----------------------------------------------------------------------------------------------/
Program used FLASH: 71.2kb(58.3%), RAM: 17.7kb(90.8%)
/----------------------------------------------------------------------------------------------/

region `SRAM' overflowed by 1484 bytes //200
region `SRAM' overflowed by 5284 bytes //300
region `SRAM' overflowed by 5604 bytes //300 + 320 byte


3800 на 100 маршрутов

38-28


3800






on: We were on. PD=1, RX=0x0002 
rf_cmd_ieee_rx: ret=0, CMDSTA=0x00000081, status=0x0000


	for(uint8_t i = 0;  i < 128; i++)
	{
		cmd_ieee_rx_buf[i] = i;
		printf(" %i", cmd_ieee_rx_buf[i]);
	}



void add_route(uint32_t serial, const uip_ip6addr_t *addr, uint16_t nonce)
{
	if(route_table_ptr >= MAX_ROUTE_TABLE) //Проверка на макс размер таблицы
		return;
	
	for(uint8_t i = 0; i < route_table_ptr; i++) //Проверка есть ли такой серийник
	{
		if(route_table[i].serial == serial)
		{
			// printf("\nSizeof: %i\n", sizeof(route_table));
			
			printf("Addr full:");
			uip_debug_ipaddr_print(addr);
			printf("\n");
			
			for(uint8_t j = 0; j < 8; j++)
				route_table[i].addr[j] = ((uint8_t *)addr)[j+8];
			
			// route_table[i].addr[0] = ((uint8_t *)addr)[8];
			// route_table[i].addr[1] = ((uint8_t *)addr)[9];
			// route_table[i].addr[2] = ((uint8_t *)addr)[10];
			// route_table[i].addr[3] = ((uint8_t *)addr)[11];
			// route_table[i].addr[4] = ((uint8_t *)addr)[12];
			// route_table[i].addr[5] = ((uint8_t *)addr)[13];
			// route_table[i].addr[6] = ((uint8_t *)addr)[14];
			// route_table[i].addr[7] = ((uint8_t *)addr)[15]; //uip_ip6addr(&addr, 0, 0, 0, 0, 0, 0, 0, 0);
			
			// printf("Addr lite:");
			// for(uint8_t j = 0; j < 8; j++)
				// printf(" %"PRIXX8, route_table[i].addr[j]);
			
			// printf("\n");
	
			
			//route_table[i].addr = addr;
			route_table[i].nonce = nonce;
			route_table[i].counter = 0xFFFF;
			//printf("Dont add serial: %lu\n", serial);
			//uip_debug_ipaddr_print(&addr);
			//printf("route_table_ptr: %i\n", route_table_ptr);
			return;
		}
	}
	
	//printf("Add serial: %lu\n", serial);
	//uip_debug_ipaddr_print(&addr);
	//printf("route_table_ptr: %i\n", route_table_ptr);
	route_table[route_table_ptr].serial = serial; //Добавляем в таблицу
	
	for(uint8_t i = 0; i < 8; i++)
		route_table[route_table_ptr].addr[i] = ((uint8_t *)addr)[i+8];
	
	//route_table[route_table_ptr].addr = addr;
	route_table[route_table_ptr].nonce = nonce;
	route_table[route_table_ptr].counter = 0xFFFF; //Добавляется в таблицу, но не будет работать, пока счетчик не обнулится
	route_table_ptr++;
	return;
}

/*---------------------------------------------------------------------------*/
uip_ip6addr_t find_addr(uint32_t serial)
{
	uip_ip6addr_t addr;
	
	for(uint8_t i = 0; i < route_table_ptr; i++)
	{
		if(route_table[i].serial == serial)
		{
			if(route_table[i].counter != 0xFFFF) //Проверка на активность.
			{
				uip_ip6addr(&addr,  
							0xFD00,
							0,
							0,
							0,
							(uint16_t)(route_table[i].addr[1] | (route_table[i].addr[0]<<8)),
							(uint16_t)(route_table[i].addr[3] | (route_table[i].addr[2]<<8)),
							(uint16_t)(route_table[i].addr[5] | (route_table[i].addr[4]<<8)),
							(uint16_t)(route_table[i].addr[7] | (route_table[i].addr[6]<<8)));
							
				return addr;
			}
		}
	}
	
	uip_ip6addr(&addr, 0, 0, 0, 0, 0, 0, 0, 0); //Адрес не найден
	return addr;
}
/*---------------------------------------------------------------------------*/
uint16_t get_nonce(uint32_t serial)
{
	for(uint8_t i = 0; i < route_table_ptr; i++)
	{
		if(route_table[i].serial == serial)
		{
			return route_table[i].nonce;
		}
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static void unlock_addr(uint32_t serial,  uint16_t counter)
{
	for(uint8_t i = 0; i < route_table_ptr; i++)
	{
		if(route_table[i].serial == serial)
		{
			if(route_table[i].counter == 0xFFFF) //Разблокируем счетчик
				route_table[i].counter = counter;
		}
	}
}
/*---------------------------------------------------------------------------*/
static bool valid_counter(uint32_t serial, uint16_t counter)
{
	for(uint8_t i = 0; i < route_table_ptr; i++)
	{
		if(route_table[i].serial == serial)
		{
			if(route_table[i].counter < counter) //Проверка на активность.
			{
				route_table[i].counter = counter;
				return true;
			}
			else
				return false;
		}
	}
	return false;
}
/*---------------------------------------------------------------------------*/












