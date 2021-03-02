clear all

figure(1)
clf
%%
%initial time t; postion x; veloctiy v; acceleration a; force F; and mass m;
t=0; x=0; v=0; a=0; F=0; m=1; 

h=plot(t,x,'Marker','square','MarkerSize',40,'MarkerFaceColor','y')
hold on
ha=plot(t,a,'r')
hv=plot(t,v,'g')
hx=plot(t,x,'b')
xlim([0,10]); ylim([-1,5]);
legend('mass','a','v','x')
grid on

%%
%start simulation
i=0; dt=0.01; mt=t;ma=a; mv=v; mx=x;
for i=1:600
    t=dt*i;

    F=1.0*(1-x);

%-----------------------------------------------------------------------------------------
% controller
    %F=t;  
    a=F/m;    v=v+a*dt; x = x+ v*dt +0.5*a*dt^2;
%-----------------------------------------------------------------------------------------    
    mt(end+1) =t; ma(end+1)=a;  mv(end+1)=v; mx(end+1)=x;
    set(h,'xdata',x,'ydata',0)
    set(ha,'xdata',mt,'ydata',ma)
    set(hv,'xdata',mt,'ydata',mv)
    set(hx,'xdata',mt,'ydata',mx)
    drawnow;                                    
    pause(dt)
end

%%
%end wait to exit
%pause(5000)