 function output = human(x,y,vx,vy)
R = [cos(atan2(vx,vy)) -sin(atan2(vx,vy)); sin(atan2(vx,vy)) cos(atan2(vx,vy))];
scale=0.3;
rhead=0.2*scale;
rbody1 = 1*scale;
rbody2 = 0.22*scale;
rnoise = 0.075*scale;
rfeet1 = 0.1*scale;
rfeet2 = 0.22*scale;
hold on
%plot head
th = 0:pi/50:2.1*pi;
head = R'*[rhead * cos(th);rhead * sin(th)]+[x;y];
h = plot(head(1,:), head(2,:),'k');

daspect([1 1 1]);

%plot body
body = R'*[rbody1 * cos(th); rbody2 * sin(th)]+[x;y];
h1 = plot(body(1,:), body(2,:),'k');
daspect([1 1 1]);

%plot noise
% Rn = [cos(pi/4) -sin(pi/4);sin(pi/4) cos(pi/4)]
noise = R'*[rnoise * cos(th);rnoise * sin(th)+rhead]+[x;y] ;
h2 = plot(noise(1,:), noise(2,:),'k');
    %plot feet1
    feet1  = R'*[rfeet1 * cos(th)-rbody1/1.7;rfeet2 * sin(th)+rhead]+[(x);(y)];
    h3 = plot(feet1(1,:),feet1(2,:),'k');
    
    %plot feet2
%     if norm([vx,vy]) ~= 0
%         feet2  = R'*[rfeet1 * cos(th)+rbody1/1.7;rfeet2 * sin(th)-rhead]+[(x);(y)];
%         h4 = plot(feet2(1,:),feet2(2,:),'k');
%     else
        feet2  = R'*[rfeet1 * cos(th)+rbody1/1.7;rfeet2 * sin(th)+rhead]+[(x);(y)];
        h4 = plot(feet2(1,:),feet2(2,:),'k');
%     end

daspect([1,1,1])
h5=0;%quiver(x,y,R(1,1),R(1,2));
if norm([vx;vy])==0
    h6=0;
else
h6=quiver(x,y,R(2,1),R(2,2),'r');
end
 alpha = linspace(0, 2*pi);
%   patch([x body(1,:) x], [y+rhead body(2,:) y+rhead], 'b','EdgeColor','b')
%  patch([x+rhead head(1,:) x+rhead], [y head(2,:) y], 'r','EdgeColor','k')
darkblue = [0 0 0.5];
p1= patch('Faces',[1:length(alpha)],'Vertices',[feet2(1,:)',feet2(2,:)'], 'FaceColor',darkblue,'EdgeColor',darkblue);
p2= patch('Faces',[1:length(alpha)],'Vertices',[feet1(1,:)',feet1(2,:)'], 'FaceColor',darkblue,'EdgeColor',darkblue);
p3= patch('Faces',[1:length(alpha)],'Vertices',[body(1,:)',body(2,:)'], 'FaceColor',[0 .5 .5],'EdgeColor','k');
p4= patch('Faces',[1:length(alpha)],'Vertices',[head(1,:)',head(2,:)'], 'FaceColor',darkblue,'EdgeColor',darkblue);
p5= patch('Faces',[1:length(alpha)],'Vertices',[noise(1,:)',noise(2,:)'], 'FaceColor',darkblue,'EdgeColor',darkblue);

output = [p1,p2,p3,p4,p5,h,h1,h2,h3,h4,h5,h6]; 