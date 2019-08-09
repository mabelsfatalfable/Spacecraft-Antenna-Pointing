clear all
close all

%A coverage

l = 45;
L = 60;

epsilon_roll = 0.05
epsilon_pitch = 0.03
epsilon_yaw = 0.5

inc = 0.03
ecc = 0.0005

NS = 0.05
EW = 0.05

delta_boresight = 0.025

re = 6378;
r0 = 35786;

%reduced or spot coverage pointing parameters

alfa = atan(re*cos(l*pi/180)*sin(L*pi/180) / (r0 + re*(1 - cos(l*pi/180)*cos(L*pi/180))))*180/pi

beta = atan(re*sin(l*pi/180)*cos(alfa*pi/180) / (r0 + re*(1 - cos(l*pi/180)*cos(L*pi/180))))*180/pi

beta_star = atan(re*sin(l*pi/180) / (r0 + re*(1 - cos(l*pi/180)*cos(L*pi/180))))*180/pi

alfa_star = atan(re*cos(l*pi/180)*sin(L*pi/180)*cos(beta*pi/180) / (r0 + re*(1 - cos(l*pi/180)*cos(L*pi/180))))*180/pi

R = sqrt(r0^2 + 2*re*(re + r0)*(1 - cos(l*pi/180)*cos(L*pi/180)))

teta = acos((r0 + re*(1 - cos(l*pi/180)*cos(L*pi/180))) / R)*180/pi

phi = atan(sin(L*pi/180)/tan(l*pi/180))*180/pi




%depointing due to attitude motion

%rotation about the roll axis

delta_teta_roll_x = atan(tan((beta_star + epsilon_roll)*pi/180)*cos(alfa*pi/180))*180/pi - beta

delta_teta_roll_y = atan(cos(beta_star*pi/180)*tan(alfa_star*pi/180)/cos((beta_star + epsilon_roll)*pi/180))*180/pi - alfa_star

%rotation about the pitch axis

delta_teta_pitch_x = atan(cos(alfa*pi/180)*tan(beta*pi/180)/cos((alfa+epsilon_pitch)*pi/180))*180/pi - beta

delta_teta_pitch_y = atan(tan((alfa + epsilon_pitch)*pi/180)*cos(beta_star*pi/180))*180/pi - alfa_star

%rotation about the yaw axis

if phi ~= 90
    
    delta_teta_yaw_x = atan(cos((phi + epsilon_yaw)*pi/180)*tan(beta*pi/180)/cos(phi*pi/180))*180/pi - beta
    
    if phi == 90
        
    delta_teta_yaw_x = -epsilon_yaw*sin(alfa*pi/180)
    
    end
    
end

delta_teta_yaw_y = atan(sin((phi + epsilon_yaw)*pi/180)*tan(alfa_star*pi/180)/sin(phi*pi/180))*180/pi - alfa_star


%depointing due to orbital motion

%latitudinal displacement (N - S)

delta_teta_NS_x = atan(tan(beta*pi/180)*(1 - cos(L*pi/180)*tan(NS*pi/180)/tan(l*pi/180)))*180/pi - beta

delta_teta_NS_y = atan(tan(alfa_star*pi/180)*(1 + tan(beta_star*pi/180)*sin(NS*pi/180)))*180/pi - alfa_star

%inclination of the orbit at nodes

if phi ~= 90
    
    delta_teta_inc_x = atan(cos((phi+inc)*pi/180)*tan(beta*pi/180)/cos(phi*pi/180))*180/pi - beta

    if phi == 90
        
    delta_teta_inc_x = -inc*sin(alfa*pi/180)
    
    end
    
end

if phi ~= 90
    
    delta_teta_inc_y = atan(sin((phi+inc)*pi/180)*tan(alfa_star*pi/180)/sin(phi*pi/180))*180/pi - alfa_star

    if phi == 90
      
    delta_teta_inc_y = inc*sin(beta_star*pi/180)
    
    end
    
end

%depointing due to east-west motion
        
delta_teta_EW_x = atan(tan(beta*pi/180)*(1 + tan(alfa*pi/180)*sin(EW*pi/180)))*180/pi - beta


if L ~= 0
    
    delta_teta_EW_y = atan(tan(alfa_star*pi/180)*(1 - tan(EW*pi/180)/tan(L*pi/180)))*180/pi - alfa_star
    
    if L == 0
        
        if l ~= 0
            
        delta_teta_EW_y = atan(tan(alfa_star*pi/180) - sin(beta_star*pi/180)*tan(EW*pi/180)/tan(l*pi/180))*180/pi - alfa_star
            
        if l == 0 
                
        delta_teta_EW_y = -(re/r0)*EW
                
        end
            
        end
        
    end
    
end

%depointing introduced by non-zero eccentricity
 
delta_teta_ecc_x = ecc*180/pi * (r0+re)/(R*cos(teta*pi/180)) * sin(beta*pi/180) * cos(beta*pi/180)

delta_teta_ecc_y = ecc*180/pi * (r0+re)/(R*cos(teta*pi/180)) * sin(alfa_star*pi/180) * cos(alfa_star*pi/180)



%overall depointing

%within station keeping box at node

delta_teta_SK_x_node = delta_teta_inc_x + delta_teta_EW_x + delta_teta_ecc_x

delta_teta_SK_y_node = delta_teta_inc_y + delta_teta_EW_y + delta_teta_ecc_y

%within station keeping box at vertex

delta_teta_SK_x_vertex = delta_teta_NS_x + delta_teta_EW_x + delta_teta_ecc_x

delta_teta_SK_y_vertex = delta_teta_NS_y + delta_teta_EW_y + delta_teta_ecc_y


%attitude control errors

delta_teta_AC_x = [delta_teta_roll_x^2 + delta_teta_pitch_x^2 + delta_teta_yaw_x^2]^0.5

delta_teta_AC_y = [delta_teta_roll_y^2 + delta_teta_pitch_y^2 + delta_teta_yaw_y^2]^0.5


%---------------------------------------------------------------

delta_teta_x_node = delta_teta_AC_x + delta_teta_SK_x_node

delta_teta_y_node = delta_teta_AC_y + delta_teta_SK_y_node


delta_teta_x_vertex = delta_teta_AC_x + delta_teta_SK_x_vertex

delta_teta_y_vertex = delta_teta_AC_y + delta_teta_SK_y_vertex

%---------------------------------------------------------------

%pessimistic approach

%at nodes

delta_teta_m_node = [delta_teta_x_node^2 + delta_teta_y_node^2]^0.5

delta_teta_m_vertex = [delta_teta_x_vertex^2 + delta_teta_y_vertex^2]^0.5

worst_depointing = max(delta_teta_m_node,delta_teta_m_vertex)


%optimistic approach 

best_at_nodes = max(delta_teta_x_node,delta_teta_y_node)

best_at_vertex = max(delta_teta_x_vertex,delta_teta_y_vertex)

best_depointing = max(best_at_nodes,best_at_vertex)

sigma = best_depointing/3


