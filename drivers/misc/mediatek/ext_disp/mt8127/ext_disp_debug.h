#ifndef EXTDEBUG_H
#define EXTDEBUG_H

extern void hdmi_log_enable(int enable);
extern void hdmi_cable_fake_plug_in(void);
extern void hdmi_cable_fake_plug_out(void);
extern void hdmi_mmp_enable(int enable);
extern void hdmi_pattern(int enable);
extern void hdmi_log_enable(int enable);
extern void init_hdmi_mmp_events(void);

void DBG_Init(void);
void DBG_Deinit(void);

#endif
