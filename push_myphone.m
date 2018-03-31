function push_myphone(message)
key = 'o.y5jfFhihFe7Pm7rbIe6eJo6P1gKtatoK';
p = Pushbullet(key);
p.pushNote([],'Matlab Pushbullet Test',message)
