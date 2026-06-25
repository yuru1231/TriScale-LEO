function beam_coverage_projection_ui()
% beam_coverage_projection_ui.m
% Interactive MATLAB version of exp_beam_coverage_projection.py
% Goal: do not generate hundreds of files; select what you want to view.
%
% Usage:
%   beam_coverage_projection_ui
%
% Notes:
%   Coverage heatmap uses numerical sampling, not Shapely polygon intersection.
%   It is intended for quick visual/debug inspection. Use the original Python
%   sweep if you need exact batch CSV outputs.

    cfg.D = 5;
    cfg.altitudeKm = 550;
    cfg.roiKm = 192;
    cfg.beamTypes = [1.0 1.5 2.0 2.5 3.0];
    cfg.beamNames = {"NARROW", "SLIGHT", "MIDDLE", "BROAD", "WIDE"};
    cfg.beamColors = [
        0.1216 0.4667 0.7059;
        1.0000 0.4980 0.0549;
        0.1725 0.6275 0.1725;
        0.8392 0.1529 0.1569;
        0.5804 0.4039 0.7412
    ];
    cfg.samplePerCellSide = 28;

    fig = uifigure('Name', 'Beam Coverage Projection Viewer', ...
                   'Position', [80 80 1320 760]);

    ax = uiaxes(fig, 'Position', [300 90 620 620]);
    ax2 = uiaxes(fig, 'Position', [950 150 330 330]);

    uilabel(fig, 'Text', 'View mode', 'Position', [25 705 120 22], 'FontWeight', 'bold');
    modeDrop = uidropdown(fig, ...
        'Items', {'Single selected beam type', 'All 5 beam types at nadir', 'Full 25-beam pattern'}, ...
        'Value', 'Full 25-beam pattern', ...
        'Position', [25 675 230 26]);

    uilabel(fig, 'Text', 'Beam type', 'Position', [25 635 120 22], 'FontWeight', 'bold');
    beamDrop = uidropdown(fig, ...
        'Items', {'NARROW 1.0 deg', 'SLIGHT 1.5 deg', 'MIDDLE 2.0 deg', 'BROAD 2.5 deg', 'WIDE 3.0 deg'}, ...
        'Value', 'MIDDLE 2.0 deg', ...
        'Position', [25 605 230 26]);

    uilabel(fig, 'Text', 'Elevation theta (deg)', 'Position', [25 560 180 22], 'FontWeight', 'bold');
    elevSlider = uislider(fig, ...
        'Limits', [37 90], 'Value', 90, ...
        'MajorTicks', [37 45 55 65 75 85 90], ...
        'Position', [35 535 210 3]);
    elevValue = uilabel(fig, 'Text', '90 deg', 'Position', [210 555 70 22]);

    uilabel(fig, 'Text', 'Azimuth phi (deg)', 'Position', [25 500 180 22], 'FontWeight', 'bold');
    azimDrop = uidropdown(fig, ...
        'Items', {'0', '45', '90', '135'}, ...
        'Value', '0', ...
        'Position', [25 470 230 26]);

    uilabel(fig, 'Text', 'Display options', 'Position', [25 425 160 22], 'FontWeight', 'bold');
    cbGrid = uicheckbox(fig, 'Text', 'Show 5x5 cell grid', 'Value', true, 'Position', [25 395 200 22]);
    cbROI = uicheckbox(fig, 'Text', 'Show ROI boundary', 'Value', true, 'Position', [25 365 200 22]);
    cbIndex = uicheckbox(fig, 'Text', 'Show cell / beam index', 'Value', true, 'Position', [25 335 200 22]);
    cbCenters = uicheckbox(fig, 'Text', 'Show beam centers', 'Value', true, 'Position', [25 305 200 22]);
    cbHeatmap = uicheckbox(fig, 'Text', 'Show coverage heatmap', 'Value', false, 'Position', [25 275 220 22]);

    uilabel(fig, 'Text', 'Parameters', 'Position', [25 230 160 22], 'FontWeight', 'bold');
    uilabel(fig, 'Text', 'Altitude km', 'Position', [25 200 85 22]);
    altEdit = uieditfield(fig, 'numeric', 'Value', cfg.altitudeKm, 'Position', [120 200 80 24]);
    uilabel(fig, 'Text', 'ROI km', 'Position', [25 165 85 22]);
    roiEdit = uieditfield(fig, 'numeric', 'Value', cfg.roiKm, 'Position', [120 165 80 24]);

    updateBtn = uibutton(fig, 'push', 'Text', 'Update plot', ...
        'Position', [25 110 105 34], 'ButtonPushedFcn', @(~,~) redraw()); %#ok<NASGU>
    exportBtn = uibutton(fig, 'push', 'Text', 'Export PNG', ...
        'Position', [150 110 105 34], 'ButtonPushedFcn', @(~,~) exportCurrent()); %#ok<NASGU>

    infoText = uitextarea(fig, 'Editable', 'off', ...
        'Position', [950 500 330 190], 'FontName', 'Consolas');

    controls = {modeDrop, beamDrop, azimDrop, cbGrid, cbROI, cbIndex, cbCenters, cbHeatmap, altEdit, roiEdit};
    for k = 1:numel(controls)
        controls{k}.ValueChangedFcn = @(~,~) redraw();
    end
    elevSlider.ValueChangingFcn = @(~,event) setElevText(event.Value);
    elevSlider.ValueChangedFcn = @(~,~) redraw();

    redraw();

    function setElevText(v)
        elevValue.Text = sprintf('%.1f deg', v);
    end

    function redraw()
        cfg.altitudeKm = altEdit.Value;
        cfg.roiKm = roiEdit.Value;
        theta = elevSlider.Value;
        elevValue.Text = sprintf('%.1f deg', theta);
        phi = str2double(azimDrop.Value);
        beamIdx = beamDropdownIndex(beamDrop.Value);
        T = cfg.beamTypes(beamIdx);

        cla(ax); cla(ax2);
        hold(ax, 'on');
        axis(ax, 'equal');
        grid(ax, 'on');
        ax.XLabel.String = 'East (km)';
        ax.YLabel.String = 'North (km)';
        ax.XLim = [-220 220];
        ax.YLim = [-220 220];

        cells = makeGridCells(cfg.roiKm, cfg.D);
        cellKm = cfg.roiKm / cfg.D;

        if cbGrid.Value
            drawGrid(ax, cells, cellKm, cbIndex.Value);
        end
        if cbROI.Value
            drawROI(ax, cfg.roiKm);
        end

        switch modeDrop.Value
            case 'Single selected beam type'
                drawBeamsAimedAtCells(ax, cells, theta, phi, T, beamIdx, false);
                ax.Title.String = sprintf('Selected beam: %s %.1f deg | theta=%.1f deg, phi=%.0f deg', ...
                    cfg.beamNames{beamIdx}, T, theta, phi);

            case 'All 5 beam types at nadir'
                for ii = 1:numel(cfg.beamTypes)
                    drawUpaPattern(ax, 90, 0, cfg.beamTypes(ii), ii, false);
                end
                ax.Title.String = sprintf('All 5 Beam Types at Nadir | h=%.0f km', cfg.altitudeKm);

            case 'Full 25-beam pattern'
                drawUpaPattern(ax, theta, phi, T, beamIdx, true);
                ax.Title.String = sprintf('Full 25-beam Pattern | %s %.1f deg | theta=%.1f deg, phi=%.0f deg', ...
                    cfg.beamNames{beamIdx}, T, theta, phi);
        end

        hold(ax, 'off');

        r = rBeamKm(T, cfg.altitudeKm);
        [a, b] = ellipseAxes(r, theta);
        roiNadir = 5 * 2 * r;
        infoText.Value = {
            sprintf('h = %.1f km', cfg.altitudeKm)
            sprintf('ROI = %.1f km, cell = %.1f km', cfg.roiKm, cellKm)
            sprintf('Beam = %s %.1f deg', cfg.beamNames{beamIdx}, T)
            sprintf('r_nadir = %.2f km', r)
            sprintf('a_along = %.2f km', a)
            sprintf('b_cross = %.2f km', b)
            sprintf('pitch along = %.2f km', 2*a)
            sprintf('pitch cross = %.2f km', 2*b)
            sprintf('natural ROI at nadir = %.2f km', roiNadir)
            sprintf('r / cell = %.3f', r/cellKm)
        };

        if cbHeatmap.Value
            Cov = computeCoverageMatrixSampling(cells, cellKm, theta, phi, T, cfg.altitudeKm, cfg.samplePerCellSide);
            imagesc(ax2, Cov, [0 1]);
            colorbar(ax2);
            axis(ax2, 'tight');
            ax2.XLabel.String = 'Cell index m';
            ax2.YLabel.String = 'Beam index n';
            ax2.Title.String = 'Cov[n,m] sampling heatmap';
        else
            ax2.Title.String = 'Coverage heatmap disabled';
            axis(ax2, 'off');
        end
    end

    function exportCurrent()
        [file, path] = uiputfile('beam_projection_current.png', 'Save current figure as PNG');
        if isequal(file, 0)
            return;
        end
        exportgraphics(fig, fullfile(path, file), 'Resolution', 200);
    end

    function drawGrid(targetAx, cells, cellKm, showIndex)
        for ii = 1:numel(cells)
            rectangle(targetAx, 'Position', [cells(ii).x0 cells(ii).y0 cellKm cellKm], ...
                'EdgeColor', [0.45 0.45 0.45], 'LineWidth', 0.8, 'FaceColor', 'none');
            if showIndex
                text(targetAx, cells(ii).cx, cells(ii).cy, sprintf('%d', cells(ii).idx), ...
                    'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', [0.35 0.35 0.35]);
            end
        end
    end

    function drawROI(targetAx, roiKm)
        half = roiKm / 2;
        rectangle(targetAx, 'Position', [-half -half roiKm roiKm], ...
            'EdgeColor', 'r', 'LineWidth', 2.2, 'FaceColor', 'none');
    end

    function drawBeamsAimedAtCells(targetAx, cells, theta, phi, T, beamIdx, labelByBeam)
        r = rBeamKm(T, cfg.altitudeKm);
        [a, b] = ellipseAxes(r, theta);
        for ii = 1:numel(cells)
            drawEllipse(targetAx, cells(ii).cx, cells(ii).cy, a, b, phi, cfg.beamColors(beamIdx,:), 0.18);
            if cbCenters.Value
                plot(targetAx, cells(ii).cx, cells(ii).cy, '.', 'Color', cfg.beamColors(beamIdx,:), 'MarkerSize', 10);
            end
            if labelByBeam && cbIndex.Value
                text(targetAx, cells(ii).cx, cells(ii).cy, sprintf('%d', cells(ii).idx), ...
                    'HorizontalAlignment', 'center', 'FontSize', 7, 'Color', cfg.beamColors(beamIdx,:));
            end
        end
    end

    function drawUpaPattern(targetAx, theta, phi, T, beamIdx, labelByBeam)
        r = rBeamKm(T, cfg.altitudeKm);
        [centers, a, b] = computeUpaBeamCenters(theta, phi, r, cfg.D);
        for ii = 1:size(centers, 1)
            cx = centers(ii, 1);
            cy = centers(ii, 2);
            drawEllipse(targetAx, cx, cy, a, b, phi, cfg.beamColors(beamIdx,:), 0.16);
            if cbCenters.Value
                plot(targetAx, cx, cy, '.', 'Color', cfg.beamColors(beamIdx,:), 'MarkerSize', 10);
            end
            if labelByBeam && cbIndex.Value
                text(targetAx, cx, cy, sprintf('%d', ii-1), ...
                    'HorizontalAlignment', 'center', 'FontSize', 7, 'Color', cfg.beamColors(beamIdx,:));
            end
        end
    end
end

function idx = beamDropdownIndex(value)
    if contains(value, 'NARROW')
        idx = 1;
    elseif contains(value, 'SLIGHT')
        idx = 2;
    elseif contains(value, 'MIDDLE')
        idx = 3;
    elseif contains(value, 'BROAD')
        idx = 4;
    else
        idx = 5;
    end
end

function r = rBeamKm(halfAngleDeg, altitudeKm)
    r = altitudeKm * tan(deg2rad(halfAngleDeg));
end

function [a, b] = ellipseAxes(rKm, elevDeg)
    s = sin(deg2rad(elevDeg));
    a = rKm / (s^2);
    b = rKm / s;
end

function [along, cross] = alongCrossUnit(azimDeg)
    phi = deg2rad(azimDeg);
    along = [sin(phi), cos(phi)];
    cross = [cos(phi), -sin(phi)];
end

function cells = makeGridCells(roiKm, D)
    cellKm = roiKm / D;
    cells = struct('idx', {}, 'row', {}, 'col', {}, 'cx', {}, 'cy', {}, 'x0', {}, 'y0', {}, 'x1', {}, 'y1', {});
    k = 1;
    for row = 0:D-1
        for col = 0:D-1
            cx = (col + 0.5 - D/2) * cellKm;
            cy = (row + 0.5 - D/2) * cellKm;
            cells(k).idx = row * D + col;
            cells(k).row = row;
            cells(k).col = col;
            cells(k).cx = cx;
            cells(k).cy = cy;
            cells(k).x0 = cx - cellKm/2;
            cells(k).y0 = cy - cellKm/2;
            cells(k).x1 = cx + cellKm/2;
            cells(k).y1 = cy + cellKm/2;
            k = k + 1;
        end
    end
end

function [centers, a, b] = computeUpaBeamCenters(elevDeg, azimDeg, rKm, D)
    [a, b] = ellipseAxes(rKm, elevDeg);
    [along, cross] = alongCrossUnit(azimDeg);
    half = floor(D / 2);
    centers = zeros(D*D, 2);
    k = 1;
    for row = 0:D-1
        for col = 0:D-1
            offset = (col - half) * 2 * b * cross + (row - half) * 2 * a * along;
            centers(k, :) = offset;
            k = k + 1;
        end
    end
end

function drawEllipse(ax, cx, cy, aAlong, bCross, azimDeg, color, alphaVal)
    t = linspace(0, 2*pi, 180);
    xLocal = bCross * cos(t);
    yLocal = aAlong * sin(t);
    [along, cross] = alongCrossUnit(azimDeg);
    pts = cx + xLocal(:) * cross(1) + yLocal(:) * along(1);
    ptn = cy + xLocal(:) * cross(2) + yLocal(:) * along(2);
    patch(ax, pts, ptn, color, 'FaceAlpha', alphaVal, 'EdgeAlpha', min(0.55, alphaVal+0.20), ...
        'EdgeColor', color, 'LineWidth', 0.7);
end

function Cov = computeCoverageMatrixSampling(cells, cellKm, elevDeg, azimDeg, halfAngleDeg, altitudeKm, samplePerSide) %#ok<INUSD>
    N = numel(cells);
    Cov = zeros(N, N);
    r = rBeamKm(halfAngleDeg, altitudeKm);
    [a, b] = ellipseAxes(r, elevDeg);
    [along, cross] = alongCrossUnit(azimDeg);

    for n = 1:N
        bc = [cells(n).cx, cells(n).cy];
        for m = 1:N
            xs = linspace(cells(m).x0, cells(m).x1, samplePerSide);
            ys = linspace(cells(m).y0, cells(m).y1, samplePerSide);
            [X, Y] = meshgrid(xs, ys);
            dx = X - bc(1);
            dy = Y - bc(2);
            uCross = dx * cross(1) + dy * cross(2);
            uAlong = dx * along(1) + dy * along(2);
            inside = (uCross ./ b).^2 + (uAlong ./ a).^2 <= 1;
            Cov(n, m) = mean(inside(:));
        end
    end
end
