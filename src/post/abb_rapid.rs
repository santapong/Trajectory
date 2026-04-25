//! ABB RAPID post-processor.
//!
//! Emits a RAPID `MODULE` containing a `PERS tooldata`, one `CONST robtarget`
//! per pose, and a `PROC main()` body of `MoveJ` (travel) and `MoveL`
//! (cutting) instructions. Quaternion order is `[w, x, y, z]` as ABB expects.

use std::fmt::Write;

use crate::toolpath::Toolpath;

use super::{PostContext, PostError, PostProcessor};

pub(crate) const MODULE_HEADER: &str = "MODULE {name}\n";
pub(crate) const MODULE_FOOTER: &str = "ENDMODULE\n";

pub struct AbbRapidPost;

impl PostProcessor for AbbRapidPost {
    fn emit(&self, path: &Toolpath, ctx: &PostContext<'_>) -> Result<String, PostError> {
        let mut out = String::new();
        let _ = write!(
            out,
            "{}",
            MODULE_HEADER.replace("{name}", ctx.program_name)
        );

        // Tool data — TCP offset, identity tool orientation, 1 kg dummy load.
        let tcp = ctx.tool.tcp.xyz;
        let _ = writeln!(
            out,
            "  PERS tooldata Tool0 := [TRUE, [[{:.3},{:.3},{:.3}],[1,0,0,0]], [1,[0,0,1],[1,0,0,0],0,0,0]];",
            tcp[0], tcp[1], tcp[2]
        );

        // Speed data
        let _ = writeln!(
            out,
            "  CONST speeddata vfeed := [{:.1},500,5000,1000];",
            ctx.feedrate_mm_min
        );
        let _ = writeln!(
            out,
            "  CONST speeddata vrapid := [{:.1},500,5000,1000];",
            ctx.rapid_speed
        );

        let xform = ctx.frame.transform();

        // Emit one CONST robtarget per pose, transformed into machine frame.
        let mut transformed: Vec<(String, [f64; 3], [f64; 4])> = Vec::with_capacity(path.poses.len());
        for (i, pose) in path.poses.iter().enumerate() {
            let p_machine = pose.transformed(xform);
            let pos = p_machine.position;
            let q = p_machine.orientation_quat();
            let quat = [q.w, q.i, q.j, q.k];
            let name = format!("P{:04}", i + 1);
            let _ = writeln!(
                out,
                "  CONST robtarget {} := [[{:.3},{:.3},{:.3}],[{:.6},{:.6},{:.6},{:.6}],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];",
                name, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]
            );
            transformed.push((name, pos, quat));
        }

        // PROC main()
        let _ = writeln!(out, "  PROC main()");
        let _ = writeln!(out, "    ConfL\\Off;");
        let _ = writeln!(out, "    ConfJ\\Off;");
        for (i, (name, _, _)) in transformed.iter().enumerate() {
            let is_feed = path.feed_flags.get(i).copied().unwrap_or(true);
            if is_feed {
                let _ = writeln!(
                    out,
                    "    MoveL {}, vfeed, fine, Tool0;",
                    name
                );
            } else {
                let _ = writeln!(
                    out,
                    "    MoveJ {}, vrapid, fine, Tool0;",
                    name
                );
            }
        }
        let _ = writeln!(out, "  ENDPROC");
        let _ = write!(out, "{}", MODULE_FOOTER);

        Ok(out)
    }
}
