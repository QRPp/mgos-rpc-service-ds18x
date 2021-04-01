#include <limits.h>

#include <mgos.h>
#include <mgos_config.h>
#include <mgos_rpc.h>

#include <mgos_arduino_dallas_temp.h>

#define SCRATCHPAD_SIZE 9

/* RPC */
#define BOOL_INVAL (sizeof(bool) > sizeof(unsigned char) ? UINT_MAX : UCHAR_MAX)
#define ERR_NBW_FMT "%s%s not within [%d..%d]"
#define ERR_NCON ERR_NCON_FMT, "", ""
#define ERR_NCON_FMT "device appears disconnected%s%s"
#define ERR_NCON_UPON ERR_NCON_FMT, " upon "

#define ADDR_OR_IDX_ "addr:%H,idx:%u"
#define ADDR_OR_IDX "{" ADDR_OR_IDX_ "}"
#define DEV_CONF_FMT_ \
  "conf:{alarm_high_C:%d,alarm_low_C:%d,resolution:%u,user_data:%d}"
#define SYS_CONF_FMT_ \
  "conf:{auto_save_scratchpad:%B,check_for_conversion:%B,resolution:%u}"
#define TEMP_POLL_FMT_ "poll:%B"
#define TEMP_POLL_FMT "{" TEMP_POLL_FMT_ "}"

#define send_errorf_exit(args...)   \
  do {                              \
    mg_rpc_send_errorf(ri, ##args); \
    goto exit;                      \
  } while (0)
#define send_errorf_return(args...) return (void) mg_rpc_send_errorf(ri, ##args)

#define scanf_dev_etc(dev, etc...)                                        \
  ({                                                                      \
    DeviceAddress *addr = NULL;                                           \
    int addrLen;                                                          \
    unsigned idx = UINT_MAX;                                              \
    int ret = json_scanf(args.p, args.len, ri->args_fmt, &addrLen, &addr, \
                         &idx, ##etc);                                    \
    if (!dev_by_addr_or_idx_p(ri, dev, addrLen, addr, idx)) return;       \
    ret;                                                                  \
  })

static bool dev_by_addr_or_idx_p(struct mg_rpc_request_info *ri,
                                 DeviceAddress *dev, int addrLen,
                                 DeviceAddress *addr, unsigned idx) {
  DallasTemperature *ds18x = NULL;
  bool ret = false;

  if (!addr + (idx == UINT_MAX) != 1)
    send_errorf_exit(400, "either addr or idx is required");

  if (addr) {
    if (addrLen != sizeof(*addr))
      send_errorf_exit(400, "addr is %d octets long, need %u", addrLen,
                       sizeof(addr));
    memcpy(*dev, *addr, sizeof(*addr));
  } else {
    if (idx > UINT8_MAX)
      send_errorf_exit(400, ERR_NBW_FMT, "", "idx", 0, UINT8_MAX);
    ds18x = mgos_ds18x_get_global_locked();
    if (!ds18x->getAddress(*dev, idx))
      send_errorf_exit(500, "no device at idx %u", idx);
  }

  ret = true;

exit:
  if (addr) free(addr);
  if (ds18x) mgos_ds18x_put_global_locked();
  return ret;
}

static int ds18x_get_dev_temp_prn(struct json_out *o, va_list *ap);

static int ds18x_get_alarms_prn(struct json_out *o, va_list *ap) {
  DeviceAddress dev;
  DallasTemperature *ds18x = va_arg(*ap, DallasTemperature *);
  bool nonFirst = false;
  int ret = 0;
  for (ds18x->resetAlarmSearch(); ds18x->alarmSearch(dev);)
    ret += json_printf(o, "%M", ds18x_get_dev_temp_prn, ds18x, &nonFirst, dev,
                       DEVICE_DISCONNECTED_RAW);
  return ret;
}

static void ds18x_get_alarms_or_temp_handler(struct mg_rpc_request_info *ri,
                                             void *cb_arg,
                                             struct mg_rpc_frame_info *fi,
                                             struct mg_str args) {
  bool poll = true;
  json_scanf(args.p, args.len, ri->args_fmt, &poll);
  DallasTemperature *ds18x = mgos_ds18x_get_global_locked();
  if (poll) {
    bool wfc = ds18x->getWaitForConversion();
    if (!wfc) ds18x->setWaitForConversion(true);
    ds18x->requestTemperatures();
    if (!wfc) ds18x->setWaitForConversion(false);
  }
  mg_rpc_send_responsef(ri, "[%M]", cb_arg, ds18x);
  mgos_ds18x_put_global_locked();
}

static void ds18x_get_dev_info_handler(struct mg_rpc_request_info *ri,
                                       void *cb_arg,
                                       struct mg_rpc_frame_info *fi,
                                       struct mg_str args) {
  DeviceAddress dev;
  scanf_dev_etc(&dev);
  DallasTemperature *ds18x = mgos_ds18x_get_global_locked();
  int8_t ha = ds18x->getHighAlarmTemp(dev);
  if (ha == DEVICE_DISCONNECTED_C) send_errorf_exit(500, ERR_NCON);
  mg_rpc_send_responsef(ri, "{addr:%H,parasite_power:%B," DEV_CONF_FMT_ "}",
                        sizeof(dev), dev, ds18x->readPowerSupply(dev), ha,
                        ds18x->getLowAlarmTemp(dev), ds18x->getResolution(dev),
                        ds18x->getUserData(dev));
exit:
  mgos_ds18x_put_global_locked();
}

static int ds18x_get_dev_temp_prn(struct json_out *o, va_list *ap) {
  DallasTemperature *ds18x = va_arg(*ap, DallasTemperature *);
  bool *nonFirst = va_arg(*ap, bool *);
  DeviceAddress *dev = va_arg(*ap, DeviceAddress *);
  int16_t t = va_arg(*ap, int);
  if (t == DEVICE_DISCONNECTED_RAW &&
      (t = ds18x->getTemp(*dev)) == DEVICE_DISCONNECTED_RAW)
    return 0;
  const char *delim = nonFirst && *nonFirst ? "," : "";
  if (nonFirst && !*nonFirst) *nonFirst = true;
  return json_printf(o, "%s{addr:%H,temp:{C:%f,F:%f,raw:%d}}", delim,
                     sizeof(*dev), *dev, ds18x->rawToCelsius(t),
                     ds18x->rawToFahrenheit(t), t);
}

static void ds18x_get_dev_temp_handler(struct mg_rpc_request_info *ri,
                                       void *cb_arg,
                                       struct mg_rpc_frame_info *fi,
                                       struct mg_str args) {
  DeviceAddress dev;
  bool poll = true;
  scanf_dev_etc(&dev, &poll);
  int16_t t;
  DallasTemperature *ds18x = mgos_ds18x_get_global_locked();
  if (poll) {
    bool wfc = ds18x->getWaitForConversion();
    if (!wfc) ds18x->setWaitForConversion(true);
    bool ret = ds18x->requestTemperaturesByAddress(dev);
    if (!wfc) ds18x->setWaitForConversion(false);
    if (!ret) send_errorf_exit(500, ERR_NCON_UPON, "temperature request");
  }
  if ((t = ds18x->getTemp(dev)) == DEVICE_DISCONNECTED_RAW)
    send_errorf_exit(500, ERR_NCON_UPON, "state readout");
  mg_rpc_send_responsef(ri, "%M", ds18x_get_dev_temp_prn, ds18x, NULL, &dev, t);

exit:
  mgos_ds18x_put_global_locked();
}

static void ds18x_get_info_handler(struct mg_rpc_request_info *ri, void *cb_arg,
                                   struct mg_rpc_frame_info *fi,
                                   struct mg_str args) {
  DallasTemperature *ds18x = mgos_ds18x_get_global_locked();
  mg_rpc_send_responsef(
      ri,
      "{one_wire_devices:%u,ds18x_devices:%u,parasite_power:%B," SYS_CONF_FMT_
      "}",
      ds18x->getDeviceCount(), ds18x->getDS18Count(),
      ds18x->isParasitePowerMode(), ds18x->getAutoSaveScratchPad(),
      ds18x->getCheckForConversion(), ds18x->getResolution());
  mgos_ds18x_put_global_locked();
}

static int ds18x_get_temp_prn(struct json_out *o, va_list *ap) {
  DallasTemperature *ds18x = va_arg(*ap, DallasTemperature *);
  bool nonFirst = false;
  int ret = 0;
  for (uint8_t idx = 0; idx < ds18x->getDeviceCount(); idx++) {
    DeviceAddress dev;
    if (!ds18x->getAddress(dev, idx)) continue;
    ret += json_printf(o, "%M", ds18x_get_dev_temp_prn, ds18x, &nonFirst, dev,
                       DEVICE_DISCONNECTED_RAW);
  }
  return ret;
}

static void ds18x_read_scratchpad_handler(struct mg_rpc_request_info *ri,
                                          void *cb_arg,
                                          struct mg_rpc_frame_info *fi,
                                          struct mg_str args) {
  DeviceAddress dev;
  scanf_dev_etc(&dev);
  uint8_t data[SCRATCHPAD_SIZE];
  if (!mgos_ds18x_get_global_locked()->readScratchPad(dev, data))
    send_errorf_exit(500, ERR_NCON);
  mg_rpc_send_responsef(ri, "{addr:%H,data:%H}", sizeof(dev), dev, sizeof(data),
                        data);
exit:
  mgos_ds18x_put_global_locked();
}

static void ds18x_recall_scratchpad_handler(struct mg_rpc_request_info *ri,
                                            void *cb_arg,
                                            struct mg_rpc_frame_info *fi,
                                            struct mg_str args) {
  DeviceAddress dev;
  scanf_dev_etc(&dev);
  if (!mgos_ds18x_get_global_locked()->recallScratchPad(dev))
    send_errorf_exit(500, ERR_NCON);
  mg_rpc_send_responsef(ri, NULL);
exit:
  mgos_ds18x_put_global_locked();
}

static void ds18x_rescan_handler(struct mg_rpc_request_info *ri, void *cb_arg,
                                 struct mg_rpc_frame_info *fi,
                                 struct mg_str args) {
  mgos_ds18x_get_global_locked()->begin();
  mgos_ds18x_put_global_locked();
  mg_rpc_send_responsef(ri, NULL);
}

static void ds18x_set_conf_handler(struct mg_rpc_request_info *ri, void *cb_arg,
                                   struct mg_rpc_frame_info *fi,
                                   struct mg_str args) {
  unsigned ass = BOOL_INVAL, cfc = BOOL_INVAL, res = UINT_MAX;
  if (json_scanf(args.p, args.len, ri->args_fmt, &ass, &cfc, &res) < 1)
    send_errorf_return(400, "conf is required");
  if (res != UINT_MAX && res > UINT8_MAX)
    send_errorf_return(400, ERR_NBW_FMT, "", "resolution", 0, UINT8_MAX);
  DallasTemperature *ds18x = mgos_ds18x_get_global_locked();
  if (ass != BOOL_INVAL) ds18x->setAutoSaveScratchPad(ass);
  if (cfc != BOOL_INVAL) ds18x->setCheckForConversion(cfc);
  if (res != UINT_MAX) ds18x->setResolution(res);
  mgos_ds18x_put_global_locked();
  mg_rpc_send_responsef(ri, NULL);
}

static void ds18x_set_dev_conf_handler(struct mg_rpc_request_info *ri,
                                       void *cb_arg,
                                       struct mg_rpc_frame_info *fi,
                                       struct mg_str args) {
  DeviceAddress dev;
  int ha = INT_MAX, la = INT_MAX, ud = INT_MAX;
  unsigned res = UINT_MAX;
  if (scanf_dev_etc(&dev, &ha, &la, &res, &ud) < 2)
    send_errorf_return(400, "conf is required");
  if ((ha != INT_MAX || la != INT_MAX) && ud != INT_MAX)
    send_errorf_return(400, "conf: %s precludes %s and %s", "user_data",
                       "alarm_high_C", "alarm_low_C");
  if (ha != INT_MAX && (ha < INT8_MIN || ha > INT8_MAX))
    send_errorf_return(400, ERR_NBW_FMT, "conf.", "alarm_high_C", INT8_MIN,
                       INT8_MAX);
  if (la != INT_MAX && (la < INT8_MIN || la > INT8_MAX))
    send_errorf_return(400, ERR_NBW_FMT, "conf.", "alarm_low_C", INT8_MIN,
                       INT8_MAX);
  if (res != UINT_MAX && res > UINT8_MAX)
    send_errorf_return(400, ERR_NBW_FMT, "conf.", "resolution", 0, UINT8_MAX);
  if (ud != INT_MAX && (ud < INT16_MIN || ud > INT16_MAX))
    send_errorf_return(400, ERR_NBW_FMT, "conf.", "user_data", INT16_MIN,
                       INT16_MAX);
  DallasTemperature *ds18x = mgos_ds18x_get_global_locked();
  if (ha != INT_MAX) ds18x->setHighAlarmTemp(dev, ha);
  if (la != INT_MAX) ds18x->setLowAlarmTemp(dev, la);
  if (res != UINT_MAX) ds18x->setResolution(dev, res);
  if (ud != INT_MAX) ds18x->setUserData(dev, ud);
  mgos_ds18x_put_global_locked();
  mg_rpc_send_responsef(ri, NULL);
}

static void ds18x_write_scratchpad_handler(struct mg_rpc_request_info *ri,
                                           void *cb_arg,
                                           struct mg_rpc_frame_info *fi,
                                           struct mg_str args) {
  DeviceAddress dev;
  uint8_t *data = NULL;
  int dataLen;
  if (scanf_dev_etc(&dev, &dataLen, &data) != 2)
    send_errorf_return(400, "data is required");
  if (dataLen != SCRATCHPAD_SIZE)
    send_errorf_exit(400, "data is %d octets long, need %u", dataLen,
                     SCRATCHPAD_SIZE);
  mgos_ds18x_get_global_locked()->writeScratchPad(dev, data);
  mgos_ds18x_put_global_locked();
  mg_rpc_send_responsef(ri, NULL);
exit:
  free(data);
}

extern "C" bool mgos_rpc_service_ds18x_init() {
  if (!mgos_ds18x_get_global() || !mgos_sys_config_get_ds18x_rpc_enable())
    return true;

  struct mg_rpc *rpc = mgos_rpc_get_global();
  bool raw_en = mgos_sys_config_get_ds18x_rpc_raw_enable();
  mg_rpc_add_handler(rpc, "DS18x.GetAlarms", TEMP_POLL_FMT,
                     ds18x_get_alarms_or_temp_handler,
                     (void *) ds18x_get_alarms_prn);
  mg_rpc_add_handler(rpc, "DS18x.GetDevInfo", ADDR_OR_IDX,
                     ds18x_get_dev_info_handler, NULL);
  mg_rpc_add_handler(rpc, "DS18x.GetDevTemp",
                     "{" ADDR_OR_IDX_ "," TEMP_POLL_FMT_ "}",
                     ds18x_get_dev_temp_handler, NULL);
  mg_rpc_add_handler(rpc, "DS18x.GetInfo", "", ds18x_get_info_handler, NULL);
  mg_rpc_add_handler(rpc, "DS18x.GetTemp", TEMP_POLL_FMT,
                     ds18x_get_alarms_or_temp_handler,
                     (void *) ds18x_get_temp_prn);
  if (raw_en) {
    mg_rpc_add_handler(rpc, "DS18x.ReadScratchpad", ADDR_OR_IDX,
                       ds18x_read_scratchpad_handler, NULL);
    mg_rpc_add_handler(rpc, "DS18x.RecallScratchpad", ADDR_OR_IDX,
                       ds18x_recall_scratchpad_handler, NULL);
  }
  mg_rpc_add_handler(rpc, "DS18x.Rescan", "", ds18x_rescan_handler, NULL);
  mg_rpc_add_handler(rpc, "DS18x.SetConf", "{" SYS_CONF_FMT_ "}",
                     ds18x_set_conf_handler, NULL);
  mg_rpc_add_handler(rpc, "DS18x.SetDevConf",
                     "{" ADDR_OR_IDX_ "," DEV_CONF_FMT_ "}",
                     ds18x_set_dev_conf_handler, NULL);
  if (raw_en)
    mg_rpc_add_handler(rpc, "DS18x.WriteScratchpad",
                       "{" ADDR_OR_IDX_ ",data:%H}",
                       ds18x_write_scratchpad_handler, NULL);
  return true;
}
