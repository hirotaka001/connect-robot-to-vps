import React from 'react';

import TestImg from 'img/VPS2DMap_310A.png';
import FootPrintImg from 'img/triangle3.png';

import { PanelProps } from '@grafana/data';
import { SimpleOptions } from 'types';
import { css, cx } from 'emotion';
import { stylesFactory, useTheme } from '@grafana/ui';

interface Props extends PanelProps<SimpleOptions> {}

export const SimplePanel: React.FC<Props> = ({ options, data, width, height }) => {
  const theme = useTheme();
  const styles = getStyles();
  // let color: string;
  // switch (options.color) {
  //   case 'red':
  //     color = theme.palette.redBase;
  //     break;
  //   case 'green':
  //     color = theme.palette.greenBase;
  //     break;
  //   case 'blue':
  //     color = theme.palette.blue95;
  //     break;
  // }

  const frames = data.series;
  const data_length = frames[0]?.length;

  // frames --- posX, posY, posZ, eulerX, eulerY, eulerZ, 0 (SELECT from Grafana)
  const v0 = frames[0]?.fields[1]?.values.get(data_length - 1);
  const v1 = frames[1]?.fields[1]?.values.get(data_length - 1);
  const v2 = frames[2]?.fields[1]?.values.get(data_length - 1);

  //const degreex = frames[3]?.fields[1]?.values.get(data_length - 1);
  const degreey = frames[4]?.fields[1]?.values.get(data_length - 1);
  //const degreez = frames[5]?.fields[1]?.values.get(data_length - 1);
  //const rotw = frames[6]?.fields[1]?.values.get(data_length - 1);

  // for display value
  let temp_value1 = 0;
  let temp_value2 = 0;

  // map pixel size
  const map_xpx = 452;
  const map_zpx = 678;

  {
    /*const frame = data.series[0];*/
  }
  return (
    <div
      className={cx(
        styles.wrapper,
        css`
          width: ${width}px;
          height: ${height}px;
        `
      )}
    >
      {(() => {
        const items = [];
        const DRAW_POINT_MAX = 300;
        let draw_points = DRAW_POINT_MAX;
        if (data_length < DRAW_POINT_MAX) {
          draw_points = data_length;
        }

        for (let i = 0; i < draw_points; ++i) {
          // 軌跡の表示数(displayCount)と透明度の計算
          let displayCount = 50;
          let sub = i - (draw_points - displayCount); // 0 < sub < displayCount)
          let opacity = sub * (1.0 / displayCount);
          if (sub < 0.0) {
            continue;
          }

          // vps data
          // pointO --- 原点
          // roomXm/Zm --- 点群の範囲(m) 点群の最大値と最小値から計算
          let pointOx = 268;
          let pointOz = 83;
          let roomXm = 10.495;
          let roomZm = 15.658;

          // VPS to 2DMap position
          let xDia = map_xpx / roomXm;
          let zDia = map_zpx / roomZm;
          let xval = pointOx + frames[0]?.fields[1]?.values.get(i) * xDia;
          let zval = pointOz + frames[2]?.fields[1]?.values.get(i) * zDia;
          let initRotOffset = 180;
          let rotation = frames[4]?.fields[1]?.values.get(i) + initRotOffset;

          temp_value1 = frames[0]?.fields[1]?.values.get(i);
          temp_value2 = xval;

          //let transStr = 'scale(0.1 0.1) rotate(0 12.5 12.5) translate(' + xval + ' ' + zval + ')';
          let transStr = 'translate(' + xval + ' ' + zval + ')' + ' rotate(' + rotation + ' 12.5 12.5) scale(1 1)';
          console.log(transStr);

          items.push(
            <g transform={transStr}>
              ); items.push(
              <image href={FootPrintImg} width="25" height="25" opacity={opacity} />
              ); items.push(
            </g>
          );
          // draw circle
          //items.push(<circle cx={xval} cy={zval} r={radius} fill={color} stroke-width="2" fill-opacity={opancity} />);
        }

        const tags = (
          <svg
            className={styles.svg}
            width={width}
            height={height}
            xmlns="http://www.w3.org/2000/svg"
            xmlnsXlink="http://www.w3.org/1999/xlink"
            viewBox={`0 0 ${width} ${height}`}
          >
            <g>
              <image href={TestImg} width={map_xpx} height={map_zpx} y="0" />
              {items}
            </g>
          </svg>
        );
        return tags;
      })()}

      <div className={styles.textBox}>
        {options.showSeriesCount && (
          <div
            className={css`
              font-size: ${theme.typography.size[options.seriesCountSize]};
            `}
          >
            Number of series: {data.series.length}
          </div>
        )}
        <div>update date 02161637: {options.text}</div>
        <div>
          width height: {width} {height}
        </div>
        <div>frame(series) count {data.series.length}</div>
        <div>data length {data_length}</div>
        <div>temp value1 {temp_value1}</div>
        <div>temp value2 {temp_value2}</div>
        <div>
          pos_xyz: {v0} {v1} {v2}{' '}
        </div>
        <div>DegreeY: {degreey} </div>
      </div>
    </div>
  );
};

const getStyles = stylesFactory(() => {
  return {
    wrapper: css`
      position: relative;
    `,
    svg: css`
      position: absolute;
      top: 0;
      left: 0;
    `,
    image: css`
      //transform: rotateZ(30deg);
    `,
    textBox: css`
      position: absolute;
      bottom: 0;
      left: 0;
      padding: 10px;
    `,
  };
});
