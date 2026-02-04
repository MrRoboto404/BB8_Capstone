% Goal: test how the 'smexportonshape' function works, and see if its
% better than manually creating mates in simulink via frames. Added bonus
% if its able to fix the global frame angle quaternion shenanigans

clear; clc; close all;
folder_path = 'C:\Users\rtsto\Projects\BB8\MATLAB Code\3D_simulation\Onshape Imports';
%% clear cad folder
delete(fullfile(folder_path, '*'));

%% load cad | WILL CREATE DUPLICATE FILES

onshape_url = 'https://cad.onshape.com/documents/1d99771202924e52fab1a4d7/w/eefaab7f081d89d76aecf5ea/e/b58d550e60d09c73df603032';

bb8_xml = smexportonshape(onshape_url, FolderPath=folder_path);

%% open multibody in simulink
smimport(bb8_xml)