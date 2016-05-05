package cz.agents.alite.trajectorytools.demo;

import java.awt.Graphics2D;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;

import javax.swing.JButton;
import javax.swing.event.MouseInputAdapter;

import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;

public class ButtonLayer extends AbstractLayer {

    public static <V extends SpatialPoint,E> VisLayer create(final String text, ActionListener actionListener, final int x, final int y, int width, int height) {
        GroupLayer group = GroupLayer.create();

        final JButton button = new JButton(text);
        button.addActionListener(actionListener);
        button.setBounds(x, y, width, height);
        button.setVisible(true);

        group.addSubLayer( new AbstractLayer() {
            MouseInputAdapter mouseListener;
            @Override
            public void init(Vis vis) {
                mouseListener = new MouseInputAdapter() {

                    @Override
                    public void mouseClicked(MouseEvent e) {
                        super.mouseClicked(e);

                        if (button.contains(e.getX() - x, e.getY() - y)) {
                            button.doClick();
                        }
                    }
                };
                vis.addMouseListener(mouseListener);
                super.init(vis);
            }

            @Override
            public void deinit(Vis vis) {
                super.deinit(vis);

                vis.removeMouseListener(mouseListener);
            }

            @Override
            public void paint(Graphics2D canvas) {
                canvas.translate(x, y);
                button.paint(canvas);
                canvas.translate(-x, -y);
            }

            @Override
            public String getLayerDescription() {
                return text;
            }
        });
        return group;
    }
}