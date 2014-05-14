function blink(a)
%a = arduino object

T=1; %period length in seconds
N=5; %number of blinks
led=13; % onboard led is on pin 13

a.pinMode(led,'output'); %initialize the digital pin as output
for i=1:N
   a.digitalWrite(13,1);
   tic 
   while (toc < T)
   end
   
   a.digitalWrite(13,0);
   tic 
   while (toc < T)
   end    
end    