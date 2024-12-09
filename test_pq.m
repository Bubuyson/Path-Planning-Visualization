% testPriorityQueue.m
clear; close all; clc;

% Create a PriorityQueue instance
pq = PriorityQueue();

% Insert some items with priorities
pq.insert([1,1], 5);
pq.insert([2,2], 3);
pq.insert([3,3], 4);

% Extract elements in order
while ~pq.isEmpty()
    [item, priority] = pq.extractMin();
    fprintf('Extracted Item: [%d, %d] with Priority: %d\n', item(1), item(2), priority);
end
