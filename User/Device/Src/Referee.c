//
// Created by CaoKangqi on 2026/2/23.
//
#include "Referee.h"

//空闲中断黏包处理函数
void Read_Data_first(ALL_RX_Data_T *ALL_RX_Data , User_Data_T *usr_data , uint16_t length)
{
	uint16_t i;
    uint16_t Data_result = 0;//数据包个数
    uint16_t Data_length;//数据长度
    ALL_RX_Data_T ALL_RX_Data_Tmp;
	for(i = 0; i<length ; i++)
	{
		if(ALL_RX_Data->Data[i] == 0xA5)
		{
						memcpy(&ALL_RX_Data_Tmp.Data[0] , &ALL_RX_Data->Data[i] , 255);
            Data_length = Read_Data_system(&ALL_RX_Data_Tmp , usr_data);

            if(Data_length < 60000)
            {
                Data_result++;
                i += Data_length+8;
            }
		}
	}
}

/**
 * @name: WHW
 * @param {uint8_t} *RX_Data
 * @param {ALL_RX_Data_T} *ALL_RX_Data
 * @performance:
 * @enter:
 * @return {0，1，2}
 */
uint16_t Read_Data_system(ALL_RX_Data_T *ALL_RX_Data , User_Data_T *user_data)
{
    if((ALL_RX_Data->RX_Data_head.frame_header.SOF == (uint16_t)0xA5)&&
			(Verify_CRC8_Check_Sum(ALL_RX_Data->Data , 5) == 1))
    {
        switch (ALL_RX_Data->RX_Data_head.read_cmd_id)
        {
            case game_state:
            {
				if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_game_status)+2) == 1)
                {
					user_data->game_status.game_type = ALL_RX_Data->RX_Data_game_status.game_status.game_type;
                    user_data->game_status.game_progress = ALL_RX_Data->RX_Data_game_status.game_status.game_progress;
                    user_data->game_status.stage_remain_time = ALL_RX_Data->RX_Data_game_status.game_status.stage_remain_time;
                    user_data->game_status.SyncTimeStamp = ALL_RX_Data->RX_Data_game_status.game_status.SyncTimeStamp;
                }
            }break;

            case Match_results:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_game_result.game_result)+2) == 1)
                {
                    user_data->game_result.winner = ALL_RX_Data->RX_Data_game_result.game_result.winner;
                }
            }break;

            case Robot_HP:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP)+2) == 1)
                {
					user_data->game_robot_HP.red_1_robot_HP     = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.red_1_robot_HP;
                    user_data->game_robot_HP.red_2_robot_HP     = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.red_2_robot_HP;
                    user_data->game_robot_HP.red_3_robot_HP     = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.red_3_robot_HP;
                    user_data->game_robot_HP.red_4_robot_HP     = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.red_4_robot_HP;
                    user_data->game_robot_HP.red_5_robot_HP     = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.red_5_robot_HP;
                    user_data->game_robot_HP.red_7_robot_HP     = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.red_7_robot_HP;
					user_data->game_robot_HP.red_outpost_HP     = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.red_outpost_HP;
                    user_data->game_robot_HP.red_base_HP        = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.red_base_HP;
                    user_data->game_robot_HP.blue_1_robot_HP    = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.blue_1_robot_HP;
                    user_data->game_robot_HP.blue_2_robot_HP    = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.blue_2_robot_HP;
                    user_data->game_robot_HP.blue_3_robot_HP    = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.blue_3_robot_HP;
                    user_data->game_robot_HP.blue_4_robot_HP    = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.blue_4_robot_HP;
                    user_data->game_robot_HP.blue_5_robot_HP    = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.blue_5_robot_HP;
                    user_data->game_robot_HP.blue_7_robot_HP    = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.blue_7_robot_HP;
					user_data->game_robot_HP.blue_outpost_HP    = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.blue_outpost_HP;
                    user_data->game_robot_HP.blue_base_HP       = ALL_RX_Data->RX_Data_game_robot_HP.game_robot_HP.blue_base_HP;
                }
            }break;

            case Venue_Events:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_event_data.event_data)+2) == 1)
                {
                    user_data->event_data.buff_big              = ALL_RX_Data->RX_Data_event_data.event_data.buff_big;
                    user_data->event_data.buff_small            = ALL_RX_Data->RX_Data_event_data.event_data.buff_small;
                    user_data->event_data.Dart_hit_aim          = ALL_RX_Data->RX_Data_event_data.event_data.Dart_hit_aim;
                    user_data->event_data.Dart_hit_time         = ALL_RX_Data->RX_Data_event_data.event_data.Dart_hit_time;
                    user_data->event_data.supply_Blood          = ALL_RX_Data->RX_Data_event_data.event_data.supply_Blood;
                    user_data->event_data.Center_Buff_RMUL      = ALL_RX_Data->RX_Data_event_data.event_data.Center_Buff_RMUL;
                    user_data->event_data.exchange_Blood        = ALL_RX_Data->RX_Data_event_data.event_data.exchange_Blood;
                    user_data->event_data.Trapezoidal_heights   = ALL_RX_Data->RX_Data_event_data.event_data.Trapezoidal_heights;
                    user_data->event_data.Receive               = ALL_RX_Data->RX_Data_event_data.event_data.Receive;
                    user_data->event_data.Center_Heights        = ALL_RX_Data->RX_Data_event_data.event_data.Center_Heights;
                    user_data->event_data.supply_Blood_RMUL     = ALL_RX_Data->RX_Data_event_data.event_data.supply_Blood_RMUL;
                }
            }break;

            case Referee_warning:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_referee_warning.referee_warning)+2) == 1)
                {
                    user_data->referee_warning.level                = ALL_RX_Data->RX_Data_referee_warning.referee_warning.level;
                    user_data->referee_warning.offending_robot_id   = ALL_RX_Data->RX_Data_referee_warning.referee_warning.offending_robot_id;
					user_data->referee_warning.count                = ALL_RX_Data->RX_Data_referee_warning.referee_warning.count;
                }
            }break;

            case Dart_fire:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_dart_info.dart_info)+2) == 1)
                {
					user_data->dart_info.dart_remaining_time    = ALL_RX_Data->RX_Data_dart_info.dart_info.dart_remaining_time;
                    user_data->dart_info.Dart_aimed             = ALL_RX_Data->RX_Data_dart_info.dart_info.Dart_aimed;
                    user_data->dart_info.Dart_aiming            = ALL_RX_Data->RX_Data_dart_info.dart_info.Dart_aiming;
                    user_data->dart_info.Dart_success           = ALL_RX_Data->RX_Data_dart_info.dart_info.Dart_success;
                    user_data->dart_info.Reserved_bits          = ALL_RX_Data->RX_Data_dart_info.dart_info.Reserved_bits;
                }
            }break;

            case Robot_performan:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_robot_status.robot_status)+2) == 1)
                {
					user_data->robot_status.robot_id                            = ALL_RX_Data->RX_Data_robot_status.robot_status.robot_id;
					user_data->robot_status.robot_level                         = ALL_RX_Data->RX_Data_robot_status.robot_status.robot_level;
					user_data->robot_status.current_HP                          = ALL_RX_Data->RX_Data_robot_status.robot_status.current_HP;
                    user_data->robot_status.maximum_HP                          = ALL_RX_Data->RX_Data_robot_status.robot_status.maximum_HP;
					user_data->robot_status.shooter_barrel_cooling_value        = ALL_RX_Data->RX_Data_robot_status.robot_status.shooter_barrel_cooling_value;
                    user_data->robot_status.shooter_barrel_heat_limit           = ALL_RX_Data->RX_Data_robot_status.robot_status.shooter_barrel_heat_limit;
					user_data->robot_status.chassis_power_limit                 = ALL_RX_Data->RX_Data_robot_status.robot_status.chassis_power_limit;
					user_data->robot_status.power_management_gimbal_output      = ALL_RX_Data->RX_Data_robot_status.robot_status.power_management_gimbal_output;
                    user_data->robot_status.power_management_chassis_output     = ALL_RX_Data->RX_Data_robot_status.robot_status.power_management_chassis_output;
                    user_data->robot_status.power_management_shooter_output     = ALL_RX_Data->RX_Data_robot_status.robot_status.power_management_shooter_output;
                }
            }break;

            case time_power:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_power_heat_data.power_heat_data)+2) == 1)
                {
					user_data->power_heat_data.chassis_voltage_reserved     = ALL_RX_Data->RX_Data_power_heat_data.power_heat_data.chassis_voltage_reserved;
					user_data->power_heat_data.chassis_current_reserved     = ALL_RX_Data->RX_Data_power_heat_data.power_heat_data.chassis_current_reserved;
					user_data->power_heat_data.chassis_power_reserved       = ALL_RX_Data->RX_Data_power_heat_data.power_heat_data.chassis_power_reserved;
                    user_data->power_heat_data.buffer_energy                = ALL_RX_Data->RX_Data_power_heat_data.power_heat_data.buffer_energy;
                    user_data->power_heat_data.shooter_17mm_1_barrel_heat   = ALL_RX_Data->RX_Data_power_heat_data.power_heat_data.shooter_17mm_1_barrel_heat;
                    user_data->power_heat_data.shooter_17mm_2_barrel_heat   = ALL_RX_Data->RX_Data_power_heat_data.power_heat_data.shooter_17mm_2_barrel_heat;
                    user_data->power_heat_data.shooter_42mm_barrel_heat     = ALL_RX_Data->RX_Data_power_heat_data.power_heat_data.shooter_42mm_barrel_heat;
                }
            }break;

            case Robot_location:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_robot_pos.robot_pos)+2) == 1)
                {
					user_data->robot_pos.x      = ALL_RX_Data->RX_Data_robot_pos.robot_pos.x;
                    user_data->robot_pos.y      = ALL_RX_Data->RX_Data_robot_pos.robot_pos.y;
                    user_data->robot_pos.angle  = ALL_RX_Data->RX_Data_robot_pos.robot_pos.angle;
                }
            }break;

            case Robot_buff:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_buff.buff)+2) == 1)
                {
					user_data->buff.recovery_buff   = ALL_RX_Data->RX_Data_buff.buff.recovery_buff;
					user_data->buff.cooling_buff    = ALL_RX_Data->RX_Data_buff.buff.cooling_buff;
					user_data->buff.defence_buff    = ALL_RX_Data->RX_Data_buff.buff.defence_buff;
                    user_data->buff.vulnerability_buff    = ALL_RX_Data->RX_Data_buff.buff.vulnerability_buff;
                    user_data->buff.attack_buff     = ALL_RX_Data->RX_Data_buff.buff.attack_buff;
                    user_data->buff.remaining_energy_50    = ALL_RX_Data->RX_Data_buff.buff.remaining_energy_50;
                    user_data->buff.remaining_energy_30    = ALL_RX_Data->RX_Data_buff.buff.remaining_energy_30;
                    user_data->buff.remaining_energy_15    = ALL_RX_Data->RX_Data_buff.buff.remaining_energy_15;
                    user_data->buff.remaining_energy_5    = ALL_RX_Data->RX_Data_buff.buff.remaining_energy_5;
                    user_data->buff.remaining_energy_1    = ALL_RX_Data->RX_Data_buff.buff.remaining_energy_1;
                    user_data->buff.remaining_energy_100    = ALL_RX_Data->RX_Data_buff.buff.remaining_energy_100;
                }
            }break;

            case Damage_status:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_hurt_data.hurt_data)+2) == 1)
                {
                    user_data->hurt_data.armor_id               = ALL_RX_Data->RX_Data_hurt_data.hurt_data.armor_id;
                    user_data->hurt_data.HP_deduction_reason    = ALL_RX_Data->RX_Data_hurt_data.hurt_data.HP_deduction_reason;
                }
            }break;

            case time_shooting:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_shoot_data.shoot_data)+2) == 1)
                {
                    user_data->shoot_data.bullet_type           = ALL_RX_Data->RX_Data_shoot_data.shoot_data.bullet_type;
					user_data->shoot_data.shooter_number        = ALL_RX_Data->RX_Data_shoot_data.shoot_data.shooter_number;
					user_data->shoot_data.launching_frequency   = ALL_RX_Data->RX_Data_shoot_data.shoot_data.launching_frequency;
                    user_data->shoot_data.initial_speed         = ALL_RX_Data->RX_Data_shoot_data.shoot_data.initial_speed;
                }
            }break;

            case Allowable_ammo:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_projectile_allowance.projectile_allowance)+2) == 1)
                {
                    user_data->projectile_allowance.projectile_allowance_17mm   = ALL_RX_Data->RX_Data_projectile_allowance.projectile_allowance.projectile_allowance_17mm;
                    user_data->projectile_allowance.projectile_allowance_42mm   = ALL_RX_Data->RX_Data_projectile_allowance.projectile_allowance.projectile_allowance_42mm;
                    user_data->projectile_allowance.remaining_gold_coin         = ALL_RX_Data->RX_Data_projectile_allowance.projectile_allowance.remaining_gold_coin;
                }
            }break;

            case RFID:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_rfid_status.rfid_status)+2) == 1)
                {
                    user_data->rfid_status.bit0     = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit0;
                    user_data->rfid_status.bit1     = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit1;
                    user_data->rfid_status.bit2     = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit2;
                    user_data->rfid_status.bit3     = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit3;
                    user_data->rfid_status.bit4     = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit4;
                    user_data->rfid_status.bit5     = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit5;
                    user_data->rfid_status.bit6     = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit6;
                    user_data->rfid_status.bit7     = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit7;
                    user_data->rfid_status.bit8     = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit8;
                    user_data->rfid_status.bit9     = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit9;
                    user_data->rfid_status.bit10    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit10;
                    user_data->rfid_status.bit11    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit11;
                    user_data->rfid_status.bit12    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit12;
                    user_data->rfid_status.bit13    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit13;
                    user_data->rfid_status.bit14    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit14;
                    user_data->rfid_status.bit15    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit15;
                    user_data->rfid_status.bit16    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit16;
                    user_data->rfid_status.bit17    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit17;
                    user_data->rfid_status.bit18    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit18;
                    user_data->rfid_status.bit19    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit19;
                    user_data->rfid_status.bit20    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit20;
                    user_data->rfid_status.bit21    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit21;
                    user_data->rfid_status.bit22    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit22;
                    user_data->rfid_status.bit23    = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit23;
                    user_data->rfid_status.bit24_31 = ALL_RX_Data->RX_Data_rfid_status.rfid_status.bit24_31;
                }
            }break;

            case Dart_directives:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_dart_client_cmd.dart_client_cmd)+2) == 1)
                {
					user_data->dart_client_cmd.dart_launch_opening_status   = ALL_RX_Data->RX_Data_dart_client_cmd.dart_client_cmd.dart_launch_opening_status;
                    user_data->dart_client_cmd.reserved                     = ALL_RX_Data->RX_Data_dart_client_cmd.dart_client_cmd.reserved;
                    user_data->dart_client_cmd.target_change_time           = ALL_RX_Data->RX_Data_dart_client_cmd.dart_client_cmd.target_change_time;
					user_data->dart_client_cmd.latest_launch_cmd_time       = ALL_RX_Data->RX_Data_dart_client_cmd.dart_client_cmd.latest_launch_cmd_time;
                }
            }break;

            case Ground_location:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_ground_robot_position.ground_robot_position)+2) == 1)
                {
					user_data->ground_robot_position.hero_x         = ALL_RX_Data->RX_Data_ground_robot_position.ground_robot_position.hero_x;
                    user_data->ground_robot_position.hero_y         = ALL_RX_Data->RX_Data_ground_robot_position.ground_robot_position.hero_y;
                    user_data->ground_robot_position.engineer_x     = ALL_RX_Data->RX_Data_ground_robot_position.ground_robot_position.engineer_x;
                    user_data->ground_robot_position.engineer_y     = ALL_RX_Data->RX_Data_ground_robot_position.ground_robot_position.engineer_y;
                    user_data->ground_robot_position.standard_3_x   = ALL_RX_Data->RX_Data_ground_robot_position.ground_robot_position.standard_3_x;
                    user_data->ground_robot_position.standard_3_y   = ALL_RX_Data->RX_Data_ground_robot_position.ground_robot_position.standard_3_y;
                    user_data->ground_robot_position.standard_4_x   = ALL_RX_Data->RX_Data_ground_robot_position.ground_robot_position.standard_4_x;
                    user_data->ground_robot_position.standard_4_y   = ALL_RX_Data->RX_Data_ground_robot_position.ground_robot_position.standard_4_y;
                    user_data->ground_robot_position.standard_5_x   = ALL_RX_Data->RX_Data_ground_robot_position.ground_robot_position.standard_5_x;
                    user_data->ground_robot_position.standard_5_y   = ALL_RX_Data->RX_Data_ground_robot_position.ground_robot_position.standard_5_y;
                }
            }break;

            case Radar_Marking:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_radar_mark_data.radar_mark_data)+2) == 1)
                {
					user_data->radar_mark_data.mark_hero_progress       = ALL_RX_Data->RX_Data_radar_mark_data.radar_mark_data.mark_hero_progress;
                    user_data->radar_mark_data.mark_engineer_progress   = ALL_RX_Data->RX_Data_radar_mark_data.radar_mark_data.mark_engineer_progress;
                    user_data->radar_mark_data.mark_standard_3_progress = ALL_RX_Data->RX_Data_radar_mark_data.radar_mark_data.mark_standard_3_progress;
                    user_data->radar_mark_data.mark_standard_4_progress = ALL_RX_Data->RX_Data_radar_mark_data.radar_mark_data.mark_standard_4_progress;
//                    user_data->radar_mark_data.mark_standard_5_progress = ALL_RX_Data->RX_Data_radar_mark_data.radar_mark_data.mark_standard_5_progress;
					user_data->radar_mark_data.mark_sentry_progress     = ALL_RX_Data->RX_Data_radar_mark_data.radar_mark_data.mark_sentry_progress;
                }
            }break;

            case Route_Informat:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_sentry_info.sentry_info)+2) == 1)
                {
                    user_data->sentry_info.remote_HP                    = ALL_RX_Data->RX_Data_sentry_info.sentry_info.remote_HP;
                    user_data->sentry_info.remote_successfully_redeemed = ALL_RX_Data->RX_Data_sentry_info.sentry_info.remote_successfully_redeemed;
                    user_data->sentry_info.Reserved_bits                = ALL_RX_Data->RX_Data_sentry_info.sentry_info.Reserved_bits;
                    user_data->sentry_info.successfully_redeemed        = ALL_RX_Data->RX_Data_sentry_info.sentry_info.successfully_redeemed;
                    user_data->sentry_info.FreeRevive                   = ALL_RX_Data->RX_Data_sentry_info.sentry_info.FreeRevive;
                    user_data->sentry_info.PayRevive                    = ALL_RX_Data->RX_Data_sentry_info.sentry_info.PayRevive;
                    user_data->sentry_info.PayReviveNum                 = ALL_RX_Data->RX_Data_sentry_info.sentry_info.PayReviveNum;
                }
            }break;

            case Radar_Informat:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_radar_info.radar_info)+2) == 1)
                {
                    user_data->radar_info.vulnerable_begin  = ALL_RX_Data->RX_Data_radar_info.radar_info.vulnerable_begin;
                    user_data->radar_info.vulnerable_now    = ALL_RX_Data->RX_Data_radar_info.radar_info.vulnerable_now;
                }
            }break;

            case Minimap:
            {
                if(Verify_CRC16_Check_Sum(ALL_RX_Data->Data , 5+2+sizeof(ALL_RX_Data->RX_Data_map_command.map_command)+2) == 1)
                {
					user_data->map_command.target_position_x    = ALL_RX_Data->RX_Data_map_command.map_command.target_position_x;
                    user_data->map_command.target_position_y    = ALL_RX_Data->RX_Data_map_command.map_command.target_position_y;
                    user_data->map_command.cmd_keyboard         = ALL_RX_Data->RX_Data_map_command.map_command.cmd_keyboard;
					user_data->map_command.target_robot_id      = ALL_RX_Data->RX_Data_map_command.map_command.target_robot_id;
                    user_data->map_command.cmd_source           = ALL_RX_Data->RX_Data_map_command.map_command.cmd_source;
                }
            }break;

            default:
                return 65534;   //没有对应的ID
        }
		ALL_RX_Data->Data[0]++;//防止处理重复数据
    }
    else
    {
        return 65533;   //校验不通过
    }
    return ALL_RX_Data->RX_Data_head.frame_header.DataLenth;

}
