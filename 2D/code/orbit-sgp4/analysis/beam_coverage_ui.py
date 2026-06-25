function beam_coverage_projection_ui_v2()
% beam_coverage_projection_ui_v2.m
% Interactive MATLAB viewer for beam footprint / ROI / 25-cell grid analysis.
%
% Main change from v1:
%   - Single Beam Analysis really draws ONE beam only.
%   - Full 25-Beam Projection draws the whole UPA 5x5 pattern.
%   - Beam Size Comparison compares 5 beam radii at nadir.
%   - Coverage Heatmap computes Cov[n,m] by numerical sampling.
%   - Sweep Analysis plots how ellipse axes change with elevation.
%
% Usage:
%   beam_coverage_projection_ui_v2

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
    cfg.samplePerCellSide = 30;

    fig = uifigure('Name', 'Beam Coverage Projection Viewer v2', ...
                   'Position', [70 70 1380 780]);

    ax = uiaxes(fig, 'Position', [310 95 650 650]);
    ax2 = uiaxes(fig, 'Position', [995 130 340 340]);

    uilabel(fig, 'Text', 'View mode', 'Position', [25 720 120 22], 'FontWeight', 'bold');
    modeDrop = uidropdown(fig, ...
        'Items', {'Single Beam Analysis', ...
                  'Full 25-Beam Projection', ...
                  'Beam Size Comparison', ...
                  'Coverage Heatmap', ...
                  'Sweep Analysis'}, ...
        'Value', 'Full 25-Beam Projection', ...
        'Position', [25 690 245 26]);

    uilabel(fig, 'Text', 'Beam type', 'Position', [25 650 120 22], 'FontWeight', 'bold');
    beamDrop = uidropdown(fig, ...
        'Items', {'NARROW 1.0 deg', 'SLIGHT 1.5 deg', 'MIDDLE 2.0 deg', 'BROAD 2.5 deg', 'WIDE 3.0 deg'}, ...
        'Value', 'MIDDLE 2.0 deg', ...
        'Position', [25 620 245 26]);

    uilabel(fig, 'Text', 'Target beam / cell index', 'Position', [25 580 170 22], 'FontWeight', 'bold');
    cellItems = arrayfun(@(x) sprintf('%d', x), 0:24, 'UniformOutput', false);
    cellDrop = uidropdown(fig, 'Items', cellItems, 'Value', '12', 'Position', [25 550 245 26]);

    uilabel(fig, 'Text', 'Elevation theta (deg)', 'Position', [25 510 180 22], 'FontWeight', 'bold');
    elevSlider = uislider(fig, ...
        'Limits', [37 90], 'Value', 90, ...
        'MajorTicks', [37 45 55 65 75 85 90], ...
        'Position', [35 485 220 3]);
    elevValue = uilabel(fig, 'Text', '90 deg', 'Position', [210 505 70 22]);

    uilabel(fig, 'Text', 'Azimuth phi (deg)', 'Position', [25 450 180 22], 'FontWeight', 'bold');
    azimDrop = uidropdown(fig, ...
        'Items', {'0', '45', '90', '135'}, ...
        'Value', '0', ...
        'Position', [25 420 245 26]);

    uilabel(fig, 'Text', 'Display options', 'Position', [25 375 160 22], 'FontWeight', 'bold');
    cbGrid = uicheckbox(fig, 'Text', 'Show 5x5 cell grid', 'Value', true, 'Position', [25 345 220 22]);
    cbROI = uicheckbox(fig, 'Text', 'Show ROI boundary', 'Value', true, 'Position', [25 315 220 22]);
    cbIndex = uicheckbox(fig, 'Text', 'Show index labels', 'Value', true, 'Position', [25 285 220 22]);
    cbCenters = uicheckbox(fig, 'Text', 'Show beam centers', 'Value', true, 'Position', [25 255 220 22]);
    cbHeatmap = uicheckbox(fig, 'Text', 'Also show Cov heatmap', 'Value', false, 'Position', [25 225 220 22]);

    uilabel(fig, 'Text', 'Parameters', 'Position', [25 180 160 22], 'FontWeight', 'bold');
    uilabel(fig, 'Text', 'Altitude km', 'Position', [25 150 85 22]);
    altEdit = uieditfield(fig, 'numeric', 'Value', cfg.altitudeKm, 'Position', [120 150 90 24]);
    uilabel(fig, 'Text', 'ROI km', 'Position', [25 115 85 22]);
    roiEdit = uieditfield(fig, 'numeric', 'Value', cfg.roiKm, 'Position', [120 115 90 24]);

    updateBtn = uibutton(fig, 'push', 'Text', 'Update plot', ...
        'Position', [25 60 110 34], 'ButtonPushedFcn', @(~,~) redraw()); %#ok<NASGU>
    exportBtn = uibutton(fig, 'push', 'Text', 'Export PNG', ...
        'Position', [160 60 110 34], 'ButtonPushedFcn', @(~,~) exportCurrent()); %#ok<NASGU>

    infoText = uitextarea(fig, 'Editable', 'off', ...
        'Position', [995 500 340 205], 'FontName', 'Consolas');

    controls = {modeDrop, beamDrop, cellDrop, azimDrop, cbGrid, cbROI, cbIndex, cbCenters, cbHeatmap, altEdit, roiEdit};
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
        selectedCell = str2double(cellDrop.Value);

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
            case 'Single Beam Analysis'
                drawSingleBeam(ax, cells, selectedCell, theta, phi, T, beamIdx);
                ax.Title.String = sprintf('Single Beam Analysis | Cell %d | %s %.1f deg | theta=%.1f deg, phi=%.0f deg', ...
                    selectedCell, cfg.beamNames{beamIdx}, T, theta, phi);
                drawSecondPanelOff('Single mode: only one selected beam is drawn.');

            case 'Full 25-Beam Projection'
                drawUpaPattern(ax, theta, phi, T, beamIdx, true);
                ax.Title.String = sprintf('Full 25-Beam Projection | %s %.1f deg | theta=%.1f deg, phi=%.0f deg', ...
                    cfg.beamNames{beamIdx}, T, theta, phi);
                drawSecondPanelOff('Full mode: complete 5x5 UPA pattern.');

            case 'Beam Size Comparison'
                drawBeamSizeComparison(ax);
                ax.Title.String = sprintf('Beam Size Comparison at Nadir | h=%.0f km | centered at Cell %d', ...
                    cfg.altitudeKm, selectedCell);
                drawSecondPanelOff('Comparison mode: 5 beam sizes at nadir.');

            case 'Coverage Heatmap'
                Cov = computeCoverageMatrixSampling(cells, cellKm, theta, phi, T, cfg.altitudeKm, cfg.samplePerCellSide);
                drawCoverageOnGrid(ax, cells, cellKm, Cov(selectedCell+1, :));
                drawSingleBeam(ax, cells, selectedCell, theta, phi, T, beamIdx);
                drawCovHeatmap(ax2, Cov);
                ax.Title.String = sprintf('Coverage Heatmap | Beam aimed at Cell %d | %s %.1f deg | theta=%.1f deg, phi=%.0f deg', ...
                    selectedCell, cfg.beamNames{beamIdx}, T, theta, phi);

            case 'Sweep Analysis'
                drawSweepAnalysis(ax, T, beamIdx);
                ax.Title.String = sprintf('Sweep Analysis | %s %.1f deg | h=%.0f km', ...
                    cfg.beamNames{beamIdx}, T, cfg.altitudeKm);
                drawSecondPanelOff('Sweep mode: ellipse axes vs elevation.');
        end

        hold(ax, 'off');

        if cbHeatmap.Value && ~strcmp(modeDrop.Value, 'Coverage Heatmap') && ~strcmp(modeDrop.Value, 'Sweep Analysis')
            Cov = computeCoverageMatrixSampling(cells, cellKm, theta, phi, T, cfg.altitudeKm, cfg.samplePerCellSide);
            drawCovHeatmap(ax2, Cov);
        end

        updateInfo(theta, phi, T, beamIdx, cellKm, selectedCell);
    end

    function updateInfo(theta, phi, T, beamIdx, cellKm, selectedCell)
        r = rBeamKm(T, cfg.altitudeKm);
        [a, b] = ellipseAxes(r, theta);
        roiNadir = 5 * 2 * r;
        infoText.Value = {
            sprintf('View = %s', modeDrop.Value)
            sprintf('h = %.1f km', cfg.altitudeKm)
            sprintf('ROI = %.1f km, cell = %.1f km', cfg.roiKm, cellKm)
            sprintf('Selected cell = %d', selectedCell)
            sprintf('Beam = %s %.1f deg', cfg.beamNames{beamIdx}, T)
            sprintf('theta = %.1f deg, phi = %.0f deg', theta, phi)
            sprintf('r_nadir = %.2f km', r)
            sprintf('a_along = %.2f km', a)
            sprintf('b_cross = %.2f km', b)
            sprintf('pitch along = %.2f km', 2*a)
            sprintf('pitch cross = %.2f km', 2*b)
            sprintf('natural ROI at nadir = %.2f km', roiNadir)
            sprintf('r / cell = %.3f', r/cellKm)
        };
    end

    function exportCurrent()
        [file, path] = uiputfile('beam_projection_current.png', 'Save current figure as PNG');
        if isequal(file, 0)
            return;
        end
        exportgraphics(fig, fullfile(path, file), 'Resolution', 200);
    end

    function drawSecondPanelOff(msg)
        cla(ax2);
        axis(ax2, 'off');
        ax2.Title.String = msg;
    end

    function drawGrid(targetAx, cells, cellKm, showIndex)
        for ii = 1:numel(cells)
            rectangle(targetAx, 'Position', [cells(ii).x0 cells(ii).y0 cellKm cellKm], ...
                'EdgeColor', [0.45 0.45 0.45], 'LineWidth', 0.8, 'FaceColor', 'none');
            if showIndex
                text(targetAx, cells(ii).cx, cells(ii).cy, sprintf('%d', cells(ii).idx), ...
                    'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', [0.30 0.30 0.30]);
            end
        end
    end

    function drawROI(targetAx, roiKm)
        half = roiKm / 2;
        rectangle(targetAx, 'Position', [-half -half roiKm roiKm], ...
            'EdgeColor', 'r', 'LineWidth', 2.2, 'FaceColor', 'none');
    end

    function drawSingleBeam(targetAx, cells, cellIdx0, theta, phi, T, beamIdx)
        r = rBeamKm(T, cfg.altitudeKm);
        [a, b] = ellipseAxes(r, theta);
        c = cells(cellIdx0 + 1);
        drawEllipse(targetAx, c.cx, c.cy, a, b, phi, cfg.beamColors(beamIdx,:), 0.28);
        if cbCenters.Value
            plot(targetAx, c.cx, c.cy, '.', 'Color', cfg.beamColors(beamIdx,:), 'MarkerSize', 18);
        end
        if cbIndex.Value
            text(targetAx, c.cx, c.cy, sprintf('Beam -> Cell %d', cellIdx0), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                'FontSize', 9, 'FontWeight', 'bold', 'Color', cfg.beamColors(beamIdx,:));
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

    function drawBeamSizeComparison(targetAx)
        cells = makeGridCells(cfg.roiKm, cfg.D);
        selectedCell = str2double(cellDrop.Value);
        c = cells(selectedCell + 1);
        for ii = 1:numel(cfg.beamTypes)
            Tloc = cfg.beamTypes(ii);
            r = rBeamKm(Tloc, cfg.altitudeKm);
            drawEllipse(targetAx, c.cx, c.cy, r, r, 0, cfg.beamColors(ii,:), 0.18);
            text(targetAx, c.cx + r, c.cy + r, sprintf('%s %.1f°', cfg.beamNames{ii}, Tloc), ...
                'FontSize', 8, 'Color', cfg.beamColors(ii,:));
        end
        if cbCenters.Value
            plot(targetAx, c.cx, c.cy, 'k.', 'MarkerSize', 14);
        end
    end

    function drawCoverageOnGrid(targetAx, cells, cellKm, covRow)
        for ii = 1:numel(cells)
            val = covRow(ii);
            colorVal = [1-val, 1, 1-val];  % white -> green-like
            rectangle(targetAx, 'Position', [cells(ii).x0 cells(ii).y0 cellKm cellKm], ...
                'EdgeColor', [0.25 0.25 0.25], 'LineWidth', 0.8, 'FaceColor', colorVal, 'FaceAlpha', 0.35);
            text(targetAx, cells(ii).cx, cells(ii).cy, sprintf('%d\n%.2f', cells(ii).idx, val), ...
                'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', [0.1 0.1 0.1]);
        end
    end

    function drawCovHeatmap(targetAx, Cov)
        cla(targetAx);
        imagesc(targetAx, Cov, [0 1]);
        colorbar(targetAx);
        axis(targetAx, 'tight');
        targetAx.XLabel.String = 'Cell index m';
        targetAx.YLabel.String = 'Beam index n';
        targetAx.Title.String = 'Cov[n,m] sampling heatmap';
    end

    function drawSweepAnalysis(targetAx, T, beamIdx)
        cla(targetAx);
        hold(targetAx, 'on');
        thetas = 37:1:90;
        r = rBeamKm(T, cfg.altitudeKm);
        aVals = zeros(size(thetas));
        bVals = zeros(size(thetas));
        for kk = 1:numel(thetas)
            [aVals(kk), bVals(kk)] = ellipseAxes(r, thetas(kk));
        end
        plot(targetAx, thetas, aVals, '-o', 'Color', cfg.beamColors(beamIdx,:), 'LineWidth', 1.8, 'MarkerSize', 3);
        plot(targetAx, thetas, bVals, '--s', 'Color', cfg.beamColors(beamIdx,:), 'LineWidth', 1.8, 'MarkerSize', 3);
        yline(targetAx, cfg.roiKm / cfg.D / 2, ':', 'Half cell size');
        xlabel(targetAx, 'Elevation theta (deg)');
        ylabel(targetAx, 'Ellipse semi-axis (km)');
        legend(targetAx, {'a along-track', 'b cross-track'}, 'Location', 'northwest');
        grid(targetAx, 'on');
        targetAx.XLim = [37 90];
        hold(targetAx, 'off');
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
