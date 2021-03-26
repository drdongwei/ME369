t=0:0.2:10;
zeta=[0.1,0.3,0.5,0.7,1];
for n=1:5
    num = [1];
    den =[1, 2*zeta(n), 1];
    sn = tf(num,den);
    y = step(sn,t);
    plot(t,y)
    hold on
end
