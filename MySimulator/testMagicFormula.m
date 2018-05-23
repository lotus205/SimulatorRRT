 alpha = linspace(-8, 8, 16);
 r = 0; 
 Fz = 5;
 a0 = 1.65;
 a1 = -34;
 a2 = 1250;
 a3 = 3036;
 a4 = 12.8;
 a5 = 0.00501;
 a6 = -0.02103;
 a7 = 0.77394;
 a8 = 0.0022890;
 a9 = 0.013442;
 a10 = 0.003709;
 a11 = 19.1656;
 a12 = 1.21356;
 a13 = 6.26206;
 C = a0;
 D = a1 * Fz^2 + a2 * Fz;
 BCD = a3 * sin(2 * atan(Fz/a4)) * (1 - a5 * abs(r));
 B = BCD / (C * D);
 Sh = a9 * Fz + a10 + a8 * r;
 k = alpha + Sh;
 Sv = a11 * Fz * r + a12 * Fz + a13;
 E = a6 * Fz^2 + a7;
 
 Fy = D * sin(C * atan(B * k - E * (B * k - atan(B * k)))) + Sv;
 figure;
 plot(alpha, Fy)
 grid
 set(gca,'xlim',[-8 8]);
 set(gca,'xtick',[-8:1:8]);
 set(gca,'ylim',[-8000 8000]);
 set(gca,'ytick',[-8000:1000:8000]);
 xlabel('slip angle');
 ylabel('lateral force/(N)');
 title('lateral force');